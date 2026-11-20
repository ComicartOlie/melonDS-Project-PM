/*
    Copyright 2016-2026 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#ifdef _WIN32
    // must precede any windows.h (pulled in by SDL) to get winsock2, not winsock1
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <shellapi.h>
    #include <wctype.h>
#endif

#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <string.h>

#include <optional>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include <string>
#include <algorithm>

#include <SDL2/SDL.h>

#include "main.h"

#include "types.h"
#include "version.h"

#include "ScreenLayout.h"

#include "Args.h"
#include "NDS.h"
#include "ARM.h"
#include "NDSCart.h"
#include "GBACart.h"
#include "GPU.h"
#include "SPU.h"
#include "Wifi.h"
#include "Platform.h"
#include "LocalMP.h"
#include "MPInterface.h"
#include "LAN.h"
#include "Config.h"
#include "RTC.h"
#include "DSi.h"
#include "DSi_I2C.h"
#include "GPU_Soft.h"
#include "GPU_OpenGL.h"

#include "Savestate.h"

#include "EmuInstance.h"
#include "MpOnline.h"

using namespace melonDS;


EmuThread::EmuThread(EmuInstance* inst, QObject* parent) : QThread(parent)
{
    emuInstance = inst;

    emuStatus = emuStatus_Paused;
    emuPauseStack = emuPauseStackRunning;
    emuActive = false;
}

void EmuThread::attachWindow(MainWindow* window)
{
    connect(this, SIGNAL(windowTitleChange(QString)), window, SLOT(onTitleUpdate(QString)));
    connect(this, SIGNAL(windowEmuStart()), window, SLOT(onEmuStart()));
    connect(this, SIGNAL(windowEmuStop()), window, SLOT(onEmuStop()));
    connect(this, SIGNAL(windowEmuPause(bool)), window, SLOT(onEmuPause(bool)));
    connect(this, SIGNAL(windowEmuReset()), window, SLOT(onEmuReset()));
    connect(this, SIGNAL(autoScreenSizingChange(int)), window->panel, SLOT(onAutoScreenSizingChanged(int)));
    connect(this, SIGNAL(windowFullscreenToggle()), window, SLOT(onFullscreenToggled()));
    connect(this, SIGNAL(screenEmphasisToggle()), window, SLOT(onScreenEmphasisToggled()));

    if (window->winHasMenu())
    {
        connect(this, SIGNAL(windowLimitFPSChange()), window->actLimitFramerate, SLOT(trigger()));
        connect(this, SIGNAL(swapScreensToggle()), window->actScreenSwap, SLOT(trigger()));
    }
}

void EmuThread::detachWindow(MainWindow* window)
{
    disconnect(this, SIGNAL(windowTitleChange(QString)), window, SLOT(onTitleUpdate(QString)));
    disconnect(this, SIGNAL(windowEmuStart()), window, SLOT(onEmuStart()));
    disconnect(this, SIGNAL(windowEmuStop()), window, SLOT(onEmuStop()));
    disconnect(this, SIGNAL(windowEmuPause(bool)), window, SLOT(onEmuPause(bool)));
    disconnect(this, SIGNAL(windowEmuReset()), window, SLOT(onEmuReset()));
    disconnect(this, SIGNAL(autoScreenSizingChange(int)), window->panel, SLOT(onAutoScreenSizingChanged(int)));
    disconnect(this, SIGNAL(windowFullscreenToggle()), window, SLOT(onFullscreenToggled()));
    disconnect(this, SIGNAL(screenEmphasisToggle()), window, SLOT(onScreenEmphasisToggled()));

    if (window->winHasMenu())
    {
        disconnect(this, SIGNAL(windowLimitFPSChange()), window->actLimitFramerate, SLOT(trigger()));
        disconnect(this, SIGNAL(swapScreensToggle()), window->actScreenSwap, SLOT(trigger()));
    }
}


// Emulated main RAM accessors for the bridge.
namespace
{

melonDS::u32 apRd32(melonDS::NDS* nds, melonDS::u32 addr)
{
    return *(melonDS::u32*)&nds->MainRAM[addr & nds->MainRAMMask];
}
melonDS::s16 apRd16s(melonDS::NDS* nds, melonDS::u32 addr)
{
    return (melonDS::s16)*(melonDS::u16*)&nds->MainRAM[addr & nds->MainRAMMask];
}
melonDS::u8 apRd8(melonDS::NDS* nds, melonDS::u32 addr)
{
    return nds->MainRAM[addr & nds->MainRAMMask];
}
void apWr32(melonDS::NDS* nds, melonDS::u32 addr, melonDS::u32 v)
{
    *(melonDS::u32*)&nds->MainRAM[addr & nds->MainRAMMask] = v;
}

}


// Mailbox bridge: syncs the ROM's multiplayer mailboxes between players each frame; the emulated radio is never touched.
namespace
{

// Cross-emulator TCP transport (host listens :7820). Frame = [u16 len LE][tag u8][role u8][size u16 LE][payload].
namespace mpnet
{
const int PORT = 7820;
const melonDS::u32 MAXFRAME = 8192;

// Wire version: the LAN beacon byte and the relay hello must always match.
const melonDS::u8 WIREVER = 2;   // 2 = 8-player rooms (7 peer slots, roles 1..8; ROM protocol v18)

// Online relay client (PMRELAY1): both sides dial out and the relay splices them into a normal game stream.
const int RELAY_PORT = MpOnlineDefaultPort;
const melonDS::u32 RELAY_RETRY_MS = 6000;    // protocol floor is 5 s
const melonDS::u32 RELAY_HS_MS = 20000;      // no reply to a hello: redial
const melonDS::u32 RELAY_MAXLINE = 200;

// UI handoff: the Qt thread posts a request and reads status; the emulation thread serves both.
std::mutex gUiMx;
struct UiRequest
{
    int want = 0;                   // 0 none, 1 host, 2 join, 3 stop
    sockaddr_in addr = {};
    char disp[96] = "";
    char name[40] = "";
    char code[8] = "";
};
UiRequest gUiReq;
MpOnlineStatus gUiStatus;

void setStr(char* dst, size_t cap, const char* src)
{
    if (!src) src = "";
    size_t n = strlen(src);
    if (n >= cap) n = cap - 1;
    memcpy(dst, src, n);
    dst[n] = 0;
}
template<size_t N> void setStr(char (&dst)[N], const char* src) { setStr(dst, N, src); }

void setNonBlock(SOCKET s)
{
    u_long nb = 1; ioctlsocket(s, FIONBIO, &nb);
    BOOL nd = TRUE; setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&nd, sizeof(nd));
}

enum { RL_DEAD = 0, RL_CONNECTING, RL_HANDSHAKE, RL_UP };

// One outbound relay connection during its handshake.
struct RelayLink
{
    SOCKET s = INVALID_SOCKET;
    int st = RL_DEAD;
    std::string out;                    // handshake bytes still to send
    std::vector<melonDS::u8> in;        // read but not yet consumed as a line
    melonDS::u32 startMs = 0;
    bool closed = false;                // relay hung up; buffered lines still count

    void close()
    {
        if (s != INVALID_SOCKET) closesocket(s);
        s = INVALID_SOCKET; st = RL_DEAD; out.clear(); in.clear(); closed = false;
    }

    bool hasLine() const
    {
        return std::find(in.begin(), in.end(), (melonDS::u8)'\n') != in.end();
    }

    // Hand the live socket to a peer slot without closing it; copy `in` out first.
    SOCKET release() { SOCKET r = s; s = INVALID_SOCKET; close(); return r; }

    bool begin(const sockaddr_in& addr, const char* hello, melonDS::u32 nowMs)
    {
        close();
        s = socket(AF_INET, SOCK_STREAM, 0);
        if (s == INVALID_SOCKET) return false;
        setNonBlock(s);
        out = hello;
        startMs = nowMs;
        int r = ::connect(s, (const sockaddr*)&addr, sizeof(addr));
        if (r == 0) st = RL_HANDSHAKE;
        else if (WSAGetLastError() == WSAEWOULDBLOCK) st = RL_CONNECTING;
        else { close(); return false; }
        return true;
    }

    // false = the link died. Never blocks.
    bool pump()
    {
        if (st == RL_DEAD) return false;
        if (st == RL_CONNECTING)
        {
            fd_set wr, ex; FD_ZERO(&wr); FD_ZERO(&ex);
            FD_SET(s, &wr); FD_SET(s, &ex);
            timeval tv = {0, 0};
            int r = select(0, NULL, &wr, &ex, &tv);
            if (r > 0 && FD_ISSET(s, &ex)) return false;
            if (r > 0 && FD_ISSET(s, &wr)) st = RL_HANDSHAKE;
            else return true;
        }
        if (!out.empty())
        {
            int r = ::send(s, out.data(), (int)out.size(), 0);
            if (r > 0) out.erase(0, r);
            else if (r == SOCKET_ERROR && WSAGetLastError() != WSAEWOULDBLOCK) return false;
        }
        char tmp[8192];
        for (;;)
        {
            int r = ::recv(s, tmp, sizeof(tmp), 0);
            if (r > 0)
            {
                in.insert(in.end(), tmp, tmp + r);
                if (in.size() > 256*1024) return false;
            }
            else if (r == 0) { closed = true; break; }  // relay hung up
            else
            {
                if (WSAGetLastError() != WSAEWOULDBLOCK) closed = true;
                break;
            }
        }
        // Report death only after buffered lines are read (the relay sends ERR and closes together).
        if (closed && !hasLine()) return false;
        return true;
    }

    // Pop one reply line. Bytes after it stay in `in`: they are game data for the peer buffer.
    bool takeLine(std::string& line)
    {
        for (size_t i = 0; i < in.size(); i++)
        {
            if (in[i] != '\n') continue;
            size_t e = i;
            if (e && in[e-1] == '\r') e--;
            line.assign((const char*)in.data(), e);
            in.erase(in.begin(), in.begin() + i + 1);
            return true;
        }
        return false;
    }

    bool overflow() const
    {
        return in.size() > RELAY_MAXLINE
            && std::find(in.begin(), in.end(), (melonDS::u8)'\n') == in.end();
    }
};

// Errors the player must fix: stop instead of retrying.
bool relayErrTerminal(const char* tok)
{
    return !strcmp(tok, "NOROOM") || !strcmp(tok, "VERSION")
        || !strcmp(tok, "FULL")   || !strcmp(tok, "BADREQ");
}

const char* relayErrText(const char* tok)
{
    if (!strcmp(tok, "NOROOM"))  return "no room with that code";
    if (!strcmp(tok, "VERSION")) return "the host runs a different version";
    if (!strcmp(tok, "FULL"))    return "the room is full";
    if (!strcmp(tok, "BADREQ"))  return "the relay rejected the request";
    if (!strcmp(tok, "RATE"))    return "relay is rate limiting, retrying";
    if (!strcmp(tok, "TIMEOUT")) return "the host did not answer, retrying";
    if (!strcmp(tok, "CLOSED"))  return "the room closed, retrying";
    return "relay error, retrying";
}

#ifdef _WIN32
// Hosting needs an inbound firewall rule; if ours is missing we ask, then add it with one admin prompt.
const wchar_t* FW_RULE = L"melonDS (Project PM)";

static void fwLower(std::wstring& s)
{
    for (size_t i = 0; i < s.size(); i++) s[i] = towlower(s[i]);
}

static bool fwRulePresent(const wchar_t* exe)
{
    HKEY k;
    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE,
        L"SYSTEM\\CurrentControlSet\\Services\\SharedAccess\\Parameters\\FirewallPolicy\\FirewallRules",
        0, KEY_READ, &k) != ERROR_SUCCESS) return false;

    // rule values are pipe-delimited token strings: v2.x|Action=Allow|...|App=path|...|Name=...|
    std::wstring wantName = L"|name="; wantName += FW_RULE; wantName += L"|";
    std::wstring wantApp  = L"|app=";  wantApp  += exe;     wantApp  += L"|";
    fwLower(wantName); fwLower(wantApp);

    bool found = false;
    wchar_t name[256]; BYTE data[4096];
    for (DWORD i = 0; !found; i++)
    {
        DWORD nl = 256, dl = sizeof(data) - 2, type = 0;
        LONG r = RegEnumValueW(k, i, name, &nl, NULL, &type, data, &dl);
        if (r == ERROR_NO_MORE_ITEMS) break;
        if (r != ERROR_SUCCESS || type != REG_SZ) continue;
        data[dl] = 0; data[dl + 1] = 0;
        std::wstring v((const wchar_t*)data);
        fwLower(v);
        found = v.find(wantName) != std::wstring::npos
             && v.find(wantApp)  != std::wstring::npos
             && v.find(L"|action=allow|") != std::wstring::npos
             && v.find(L"|dir=in|")       != std::wstring::npos
             && v.find(L"|active=true|")  != std::wstring::npos;
    }
    RegCloseKey(k);
    return found;
}

static void ensureFirewall()
{
    static bool once = false;
    if (once) return;
    once = true;

    wchar_t exe[MAX_PATH];
    DWORD n = GetModuleFileNameW(NULL, exe, MAX_PATH);
    if (!n || n >= MAX_PATH) return;

    if (fwRulePresent(exe)) { printf("[BR] firewall rule ok\n"); return; }

    if (MessageBoxW(NULL,
        L"To let friends join your hosted game, Windows Firewall needs an\n"
        L"inbound rule for this emulator (without it, players outside your\n"
        L"PC usually can't connect even with the ports forwarded).\n\n"
        L"Add the rule now?  Windows will show one administrator prompt.",
        L"Project PM - enable hosting", MB_YESNO | MB_ICONQUESTION) != IDYES)
    {
        printf("[BR] firewall rule declined by user\n");
        return;
    }

    wchar_t args[1200];
    swprintf(args, 1200,
        L"/c netsh advfirewall firewall delete rule name=all program=\"%ls\" & "
        L"netsh advfirewall firewall add rule name=\"%ls\" dir=in action=allow "
        L"program=\"%ls\" enable=yes profile=any", exe, FW_RULE, exe);
    HINSTANCE rc = ShellExecuteW(NULL, L"runas", L"cmd.exe", args, NULL, SW_SHOWMINNOACTIVE);
    printf("[BR] firewall rule %s\n",
        ((INT_PTR)rc > 32) ? "repair launched" : "not added (admin prompt declined)");
}
#else
static void ensureFirewall() {}
#endif

struct Peer
{
    SOCKET s = INVALID_SOCKET;
    bool up = false;
    bool reserved = false;      // online host: an accept connection is in flight for this slot
    std::vector<melonDS::u8> rx, tx;
};

struct Net
{
    int mode = 0;                       // 0 off, 1 host, 2 join
    char joinIP[64] = "127.0.0.1";
    bool started = false;
    SOCKET listener = INVALID_SOCKET;
    Peer peers[7];                      // host: slot i = role 2+i; join: peers[0] = host link
    bool connecting = false;
    bool freshPeer = false;     // link just came up: resend party/pkt next pump
    melonDS::u32 retryAt = 0;
    int assignedRole = 0;               // join: role handed out by the host

    // ---- online relay session (0 = plain LAN transport, unchanged) ----
    int online = 0;                     // 0 off, 1 host via relay, 2 join via relay
    sockaddr_in relayAddr = {};         // resolved once, never in the frame loop
    char relaySrv[96] = "";             // "host:port" as the player typed it
    char roomCode[8] = "";              // host: assigned by the relay; join: target
    char lobbyName[40] = "Player";
    char onlineMsg[128] = "";           // connection state / last error, for the UI
    bool onlineFatal = false;           // terminal error: stop, do not hammer the relay
    melonDS::u32 onlineRetryMs = 0;
    melonDS::u32 onlineLogMs = 0;
    melonDS::u32 ctlSeenMs = 0;         // last line from the relay control link
    RelayLink ctl;                      // host control connection
    RelayLink jlink;                    // joiner connection (becomes peers[0])
    struct AcceptLink { RelayLink l; int slot = -1; int ticket = 0; };
    AcceptLink acc[7];

    // Lobby frames: name [0xFE][role][ping u16][len][name], ping [0xFD][role][tick u32], pong [0xFC] likewise.
    char myName[24] = "Player";
    bool nameSet = false;
    char rname[9][24] = {{0}};          // display name by role (1..8)
    melonDS::u16 rping[9] = {0};        // that role's ping to the host, ms
    melonDS::u16 myPingMs = 0;
    melonDS::u32 pingSentAt = 0;
    melonDS::u32 pongRx = 0;            // pongs the host answered
    melonDS::u32 lastNameF = 0, lastPingF = 0;
    melonDS::u32 lobbySeen[9] = {0};    // LOBBY presence, by role.

    int myRole() const { return (mode == 1) ? 1 : (assignedRole ? assignedRole : 2); }
    // Host: slot i may only speak as role 2+i. Joiners trust the host's relay.
    bool roleAllowed(int slot, int role) const { return mode != 1 || role == 2 + slot; }
    bool anyUp() const { for (int i=0;i<7;i++) if (peers[i].up) return true; return false; }
    int upCount() const { int n=0; for (int i=0;i<7;i++) if (peers[i].up) n++; return n; }

    void setMsg(const char* m) { setStr(onlineMsg, m); }

    void setName(const char* nm)
    {
        if (nm && nm[0]) { setStr(myName, nm); nameSet = true; }
    }

    // Falls back to the PC name, matching what the LAN beacon advertises.
    void ensureName()
    {
        if (nameSet) return;
        nameSet = true;
        char nm[24]; DWORD n = 24;
        if (GetComputerNameA(nm, &n)) setStr(myName, nm);
    }

    void sendName(int role)
    {
        melonDS::u8 buf[29];
        int nl = (int)strlen(myName); if (nl > 23) nl = 23;
        buf[0] = 0xFE; buf[1] = (melonDS::u8)role;
        buf[2] = (melonDS::u8)(myPingMs & 0xFF); buf[3] = (melonDS::u8)(myPingMs >> 8);
        buf[4] = (melonDS::u8)nl;
        memcpy(buf + 5, myName, nl);
        sendAll(buf, 5 + nl);
        if (role >= 1 && role <= 8)
        {
            memcpy(rname[role], myName, nl); rname[role][nl] = 0;
            rping[role] = myPingMs;
        }
    }

    void sendPing(int role)     // clients measure their ping to the host
    {
        if (mode != 2 || !peers[0].up) return;
        pingSentAt = GetTickCount();
        melonDS::u8 buf[6] = { 0xFD, (melonDS::u8)role,
            (melonDS::u8)(pingSentAt & 0xFF), (melonDS::u8)((pingSentAt >> 8) & 0xFF),
            (melonDS::u8)((pingSentAt >> 16) & 0xFF), (melonDS::u8)((pingSentAt >> 24) & 0xFF) };
        enqueue(0, buf, 6);
    }

    // Handles control frames; true = consumed. Call before the game-bundle path.
    bool handleCtlFrame(int pi, const melonDS::u8* rx, melonDS::u32 n, int role)
    {
        if (n == 2 && rx[0] == 0xFF)                    // host-assigned role
        {
            // only a joiner takes it, and only from the host link
            if (mode != 2 || pi != 0 || rx[1] < 2 || rx[1] > 8) return true;
            if (assignedRole != rx[1])
            {
                printf("[BR] host assigned role %d\n", rx[1]);
                fflush(stdout);
            }
            assignedRole = rx[1];
            sendName(myRole());                         // announce ourselves at once
            return true;
        }
        if (n == 6 && rx[0] == 0xFD)                    // ping: echo it back as a pong
        {
            melonDS::u8 pong[6]; memcpy(pong, rx, 6); pong[0] = 0xFC;
            enqueue(pi, pong, 6);
            return true;
        }
        if (n == 6 && rx[0] == 0xFC)                    // pong: round trip is our ping
        {
            melonDS::u32 tick = rx[2] | (rx[3] << 8) | (rx[4] << 16) | ((melonDS::u32)rx[5] << 24);
            melonDS::u32 rtt = GetTickCount() - tick;
            myPingMs = (rtt > 9999) ? 9999 : (melonDS::u16)rtt;
            pongRx++;
            return true;
        }
        if (n >= 5 && rx[0] == 0xFE)                    // lobby name announce
        {
            int r = rx[1];
            melonDS::u16 png = (melonDS::u16)(rx[2] | (rx[3] << 8));
            int nl = rx[4];
            if (r >= 1 && r <= 8 && nl <= 23 && n >= (melonDS::u32)(5 + nl)
                && roleAllowed(pi, r))
            {
                // names come from other machines: blank out control characters
                char nm[24]; memcpy(nm, rx + 5, nl); nm[nl] = 0;
                for (char* c = nm; *c; c++) if ((unsigned char)*c < 0x20 || *c == 0x7F) *c = '?';
                bool isNew = (lobbySeen[r] == 0) || strcmp(rname[r], nm) != 0;
                memcpy(rname[r], nm, nl + 1);
                rping[r] = png;
                lobbySeen[r] = GetTickCount();
                if (isNew)
                {
                    printf("[BR] lobby name: role %d = \"%s\" (ping %u ms)\n", r, rname[r], png);
                    fflush(stdout);
                }
                // Host relay: pass a client's name on to the other clients.
                if (myRole() == 1)
                    for (int pj = 0; pj < 7; pj++)
                        if (pj != pi) enqueue(pj, rx, n);
            }
            return true;
        }
        return false;
    }

    // Peeks the queued head frame without consuming it.  0 = nothing complete.
    melonDS::u32 peekFrame(int i, melonDS::u8& tag) const
    {
        const Peer& p = peers[i];
        if (p.rx.size() < 2) return 0;
        melonDS::u32 n = (melonDS::u32)p.rx[0] | ((melonDS::u32)p.rx[1] << 8);
        if (n == 0 || n > MAXFRAME) return 0;       // recvFrame drops the peer
        if (p.rx.size() < 2 + n) return 0;
        tag = p.rx[2];
        return n;
    }

    // Drain control frames every frame, even before the ROM is up; stops at the first game bundle.
    void drainControl(int pi)
    {
        for (int guard = 0; guard < 32; guard++)
        {
            melonDS::u8 tag = 0;
            melonDS::u32 n = peekFrame(pi, tag);
            if (!n || n > 64) return;
            if (tag != 0xFF && tag != 0xFE && tag != 0xFD && tag != 0xFC) return;
            melonDS::u8 buf[64];
            melonDS::u32 got = recvFrame(pi, buf, sizeof(buf));
            if (!got) return;
            handleCtlFrame(pi, buf, got, myRole());
        }
    }

    // Send our name twice a second; clients also ping the host.
    void lobbyTick(melonDS::u32 frame)
    {
        for (int i = 0; i < 7; i++) drainControl(i);
        if (!anyUp()) return;
        if ((mode == 1 || assignedRole) && frame - lastNameF >= 30)
        {
            lastNameF = frame;
            sendName(myRole());
        }
        if (mode == 2 && frame - lastPingF >= 30)
        {
            lastPingF = frame;
            sendPing(myRole());
        }
    }

    void startHost()
    {
        ensureFirewall();
        ensureName();
        WSADATA w; WSAStartup(MAKEWORD(2,2), &w);
        mode = 1; started = true;
        listener = socket(AF_INET, SOCK_STREAM, 0);
        if (listener == INVALID_SOCKET) { mode = 0; return; }
        BOOL yes = TRUE;
        setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes));
        sockaddr_in a; memset(&a, 0, sizeof(a));
        a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons(PORT);
        if (bind(listener, (sockaddr*)&a, sizeof(a)) != 0 || listen(listener, 3) != 0)
        {
            closesocket(listener); listener = INVALID_SOCKET; mode = 0; return;
        }
        setNonBlock(listener);
        printf("[BR] hosting on :%d\n", PORT);
    }

    void startJoin(const char* ip)
    {
        ensureName();
        WSADATA w; WSAStartup(MAKEWORD(2,2), &w);
        if (ip && ip[0]) { strncpy(joinIP, ip, sizeof(joinIP)-1); joinIP[sizeof(joinIP)-1] = 0; }
        mode = 2; started = true;
    }

    // Online host: outbound only, so no listener, beacon or firewall prompt.
    void startHostOnline(const sockaddr_in& addr, const char* disp, const char* name)
    {
        WSADATA w; WSAStartup(MAKEWORD(2,2), &w);
        mode = 1; online = 1; started = true;
        relayAddr = addr;
        setStr(relaySrv, disp);
        if (name && name[0]) setStr(lobbyName, name);
        setName(lobbyName); ensureName();
        roomCode[0] = 0; onlineFatal = false; onlineRetryMs = 0;
        setMsg("connecting to the relay...");
        printf("[BR] online host: relay %s, name \"%s\", wire ver %d\n",
            relaySrv, lobbyName, (int)WIREVER);
        fflush(stdout);
    }

    void startJoinOnline(const sockaddr_in& addr, const char* disp,
                         const char* code, const char* name)
    {
        WSADATA w; WSAStartup(MAKEWORD(2,2), &w);
        mode = 2; online = 2; started = true;
        relayAddr = addr;
        setStr(relaySrv, disp);
        if (name && name[0]) setStr(lobbyName, name);
        setName(lobbyName); ensureName();
        onlineFatal = false; onlineRetryMs = 0;
        roomCode[0] = 0;
        if (code && code[0]) { setStr(roomCode, code); upperCode(); }
        setMsg("connecting...");
        printf("[BR] online join: relay %s, room %s, name \"%s\"\n",
            relaySrv, roomCode, lobbyName);
        fflush(stdout);
    }

    void upperCode()
    {
        for (char* p = roomCode; *p; p++)
            if (*p >= 'a' && *p <= 'z') *p = (char)(*p - 'a' + 'A');
    }

    // LAN discovery beacon (UDP :7821), so DeSmuME and BizHawk can find a melonDS host.
    SOCKET beaconTx = INVALID_SOCKET;
    melonDS::u32 lastBeacon = 0;
    void beaconTick(melonDS::u32 frame, melonDS::u8 players)
    {
        if (mode != 1) return;
        if (online) return;     // relayed room: no LAN peer could use this advert
        if (beaconTx == INVALID_SOCKET)
        {
            beaconTx = socket(AF_INET, SOCK_DGRAM, 0);
            if (beaconTx == INVALID_SOCKET) return;
            BOOL b = TRUE; setsockopt(beaconTx, SOL_SOCKET, SO_BROADCAST, (const char*)&b, sizeof(b));
        }
        if (frame - lastBeacon < 60) return;
        lastBeacon = frame;
        melonDS::u8 buf[32]; memset(buf, 0, sizeof(buf));
        memcpy(buf, "PLATMP", 6); buf[6] = WIREVER; buf[7] = players;
        char nm[24]; DWORD n = 24; if (!GetComputerNameA(nm, &n)) { strcpy(nm, "melonDS"); n = 7; }
        memcpy(buf + 8, nm, (n < 24) ? n : 23);
        sockaddr_in a; memset(&a, 0, sizeof(a));
        a.sin_family = AF_INET; a.sin_port = htons(7821); a.sin_addr.s_addr = INADDR_BROADCAST;
        sendto(beaconTx, (const char*)buf, 32, 0, (sockaddr*)&a, sizeof(a));
    }

    void dropPeer(int i)
    {
        Peer& p = peers[i];
        if (p.s != INVALID_SOCKET) closesocket(p.s);
        p.s = INVALID_SOCKET; p.up = false; p.reserved = false;
        p.rx.clear(); p.tx.clear();
        if (mode == 2) connecting = false;
        if (online == 2 && i == 0)
        {
            // Relayed link died: redial the room after the protocol's backoff.
            onlineRetryMs = SDL_GetTicks() + RELAY_RETRY_MS;
            setMsg("link lost, reconnecting");
            printf("[BR] online join: link lost, redialling room %s\n", roomCode);
            fflush(stdout);
        }
    }

    // ---- online relay state machines ---------------------------------------

    // Apply an "ERR <token>" reply: terminal ones stop the session, the rest back off and retry.
    void applyErr(const char* tok, melonDS::u32 now, const char* who)
    {
        char m[128];
        snprintf(m, sizeof(m), "%s", relayErrText(tok));
        setMsg(m);
        if (relayErrTerminal(tok))
        {
            onlineFatal = true;
            printf("[BR] online %s: %s (ERR %s) - stopping\n", who, m, tok);
        }
        else
        {
            onlineRetryMs = now + RELAY_RETRY_MS;
            printf("[BR] online %s: %s (ERR %s)\n", who, m, tok);
        }
        fflush(stdout);
    }

    void startAccept(int tid, melonDS::u32 now)
    {
        int slot = -1, ai = -1;
        for (int i = 0; i < 7; i++)
            if (!peers[i].up && !peers[i].reserved) { slot = i; break; }
        for (int i = 0; i < 7; i++)
            if (acc[i].l.st == RL_DEAD) { ai = i; break; }
        if (slot < 0 || ai < 0)
        {
            // No free slot: leave the ticket unclaimed so the relay times it out.
            printf("[BR] online host: ticket %d ignored, no free slot\n", tid);
            fflush(stdout);
            return;
        }
        char hello[128];
        snprintf(hello, sizeof(hello), "PMRELAY1 ACCEPT %s %d\n", roomCode, tid);
        if (!acc[ai].l.begin(relayAddr, hello, now)) return;
        acc[ai].slot = slot; acc[ai].ticket = tid;
        peers[slot].reserved = true;
        printf("[BR] online host: ticket %d -> accept connection (slot %d, role %d)\n",
            tid, slot, 2 + slot);
        fflush(stdout);
    }

    // Spliced: install the socket in the peer slot like an accepted LAN client, leftover bytes included.
    void installLink(RelayLink& l, int i)
    {
        Peer& p = peers[i];
        p.rx.assign(l.in.begin(), l.in.end());      // leftover-buffer rule
        p.tx.clear();
        p.s = l.release();
        p.up = true; p.reserved = false;
        connecting = false;
        freshPeer = true;
    }

    void acceptTick(int i, melonDS::u32 now)
    {
        AcceptLink& a = acc[i];
        if (a.l.st == RL_DEAD) return;
        bool bad = !a.l.pump() || a.l.overflow();
        if (!bad && a.l.st != RL_UP && now - a.l.startMs > RELAY_HS_MS)
        {
            printf("[BR] online host: ticket %d accept timed out\n", a.ticket);
            fflush(stdout);
            bad = true;
        }
        std::string line;
        if (!bad && a.l.takeLine(line))
        {
            char v[32] = {0}, t[64] = {0};
            sscanf(line.c_str(), "%31s %63s", v, t);
            if (!strcmp(v, "OK"))
            {
                melonDS::u32 leftover = (melonDS::u32)a.l.in.size();
                int slot = a.slot;
                installLink(a.l, slot);
                melonDS::u8 ctlf[4] = { 2, 0, 0xFF, (melonDS::u8)(2 + slot) };
                peers[slot].tx.insert(peers[slot].tx.end(), ctlf, ctlf + 4);
                flush(slot);
                printf("[BR] peer accepted (online ticket %d) -> role %d (%u leftover byte(s))\n",
                    a.ticket, 2 + slot, leftover);
                fflush(stdout);
                a.slot = -1; a.ticket = 0;
                return;
            }
            printf("[BR] online host: ticket %d rejected (%s %s)\n", a.ticket, v, t);
            fflush(stdout);
            bad = true;
        }
        if (bad)
        {
            if (a.slot >= 0) peers[a.slot].reserved = false;
            a.slot = -1;
            a.l.close();
        }
    }

    void hostOnlineTick(melonDS::u32 now)
    {
        for (int i = 0; i < 7; i++) acceptTick(i, now);

        if (ctl.st == RL_DEAD)
        {
            if (onlineFatal || now < onlineRetryMs) return;
            char hello[300];
            snprintf(hello, sizeof(hello), "PMRELAY1 HOST %d %s\n", (int)WIREVER, lobbyName);
            if (!ctl.begin(relayAddr, hello, now))
            {
                onlineRetryMs = now + RELAY_RETRY_MS;
                setMsg("relay unreachable, retrying");
                return;
            }
            roomCode[0] = 0;
            ctlSeenMs = now;
            setMsg("connecting to the relay...");
            return;
        }

        bool bad = !ctl.pump() || ctl.overflow();
        bool said = false;      // an ERR reply already explained itself
        if (!bad && ctl.st != RL_UP && now - ctl.startMs > RELAY_HS_MS) bad = true;
        // the relay pings every 30 s, so long silence means the room is gone
        if (!bad && ctl.st == RL_UP && now - ctlSeenMs > 120000) bad = true;
        std::string line;
        while (!bad && ctl.takeLine(line))
        {
            ctlSeenMs = now;
            char v[32] = {0}, t[64] = {0};
            sscanf(line.c_str(), "%31s %63s", v, t);
            if (!strcmp(v, "OK") && t[0])
            {
                ctl.st = RL_UP;
                setStr(roomCode, t);
                char m[128];
                snprintf(m, sizeof(m), "room code %s on %s", roomCode, relaySrv);
                setMsg(m);
                printf("[BR] online room code %s on %s\n", roomCode, relaySrv);
                fflush(stdout);
            }
            else if (!strcmp(v, "JOIN")) startAccept(atoi(t), now);
            else if (!strcmp(v, "PING")) ctl.out += "PONG\n";
            else if (!strcmp(v, "ERR"))
            {
                applyErr(t, now, "host");
                bad = true; said = true;
            }
        }

        if (bad)
        {
            ctl.close();
            roomCode[0] = 0;
            if (!onlineFatal && !said)
            {
                onlineRetryMs = now + RELAY_RETRY_MS;
                setMsg("relay link lost, reconnecting");
                printf("[BR] online host: relay link lost, reconnecting\n");
                fflush(stdout);
            }
        }
    }

    void joinOnlineTick(melonDS::u32 now)
    {
        if (peers[0].up) return;                // spliced: normal joiner behaviour

        if (jlink.st == RL_DEAD)
        {
            if (onlineFatal || now < onlineRetryMs) return;
            char hello[300];
            snprintf(hello, sizeof(hello), "PMRELAY1 JOIN %s %d %s\n",
                roomCode, (int)WIREVER, lobbyName);
            if (!jlink.begin(relayAddr, hello, now))
            {
                onlineRetryMs = now + RELAY_RETRY_MS;
                setMsg("relay unreachable, retrying");
                return;
            }
            char m[128]; snprintf(m, sizeof(m), "room %s (connecting...)", roomCode);
            setMsg(m);
            printf("[BR] online join: dialling room %s via %s\n", roomCode, relaySrv);
            fflush(stdout);
            return;
        }

        bool bad = !jlink.pump() || jlink.overflow();
        bool said = false;
        if (!bad && now - jlink.startMs > RELAY_HS_MS)
        {
            setMsg("the relay did not answer, retrying");
            printf("[BR] online join: no reply from the relay, retrying\n");
            fflush(stdout);
            bad = true;
        }
        std::string line;
        if (!bad && jlink.takeLine(line))
        {
            char v[32] = {0}, t[64] = {0};
            sscanf(line.c_str(), "%31s %63s", v, t);
            if (!strcmp(v, "OK"))
            {
                melonDS::u32 leftover = (melonDS::u32)jlink.in.size();
                installLink(jlink, 0);
                char m[128]; snprintf(m, sizeof(m), "room %s (connected)", roomCode);
                setMsg(m);
                printf("[BR] online join: spliced into room %s (%u leftover byte(s))\n",
                    roomCode, leftover);
                fflush(stdout);
                return;
            }
            if (!strcmp(v, "ERR")) { applyErr(t, now, "join"); said = true; }
            bad = true;
        }
        if (bad)
        {
            jlink.close();
            if (!onlineFatal && !said) onlineRetryMs = now + RELAY_RETRY_MS;
        }
    }

    void onlineTick()
    {
        melonDS::u32 now = SDL_GetTicks();
        if (online == 1) hostOnlineTick(now);
        else if (online == 2) joinOnlineTick(now);

        // Periodic lobby status line.
        if (now - onlineLogMs >= 10000)
        {
            onlineLogMs = now;
            // show a role frame still waiting in the receive buffer
            const Peer& p0 = peers[0];
            int pendRole = (online == 2 && p0.rx.size() >= 4
                && p0.rx[0] == 2 && p0.rx[1] == 0 && p0.rx[2] == 0xFF) ? p0.rx[3] : 0;
            printf("[BR] online %s: room %s, state \"%s\", links=%d, pendingRole=%d, pongs=%u\n",
                (online == 1) ? "host" : "join",
                roomCode[0] ? roomCode : "-", onlineMsg, upCount(), pendRole, pongRx);
            printf("[BR] lobby roster (me = role %d):", myRole());
            for (int r = 1; r <= 8; r++)
                if (rname[r][0]) printf(" [%d]\"%s\" %ums", r, rname[r], rping[r]);
            printf("\n");
            fflush(stdout);
        }

        publishStatus();
    }

    void publishStatus()
    {
        std::lock_guard<std::mutex> lk(gUiMx);
        gUiStatus.mode = online;
        gUiStatus.peers = upCount();
        gUiStatus.pending = (gUiReq.want != 0);
        setStr(gUiStatus.code, roomCode);
        setStr(gUiStatus.server, relaySrv);
        setStr(gUiStatus.text, onlineMsg);
        gUiStatus.myRole = myRole();
        for (int r = 1; r <= 8; r++)
        {
            setStr(gUiStatus.roster[r], rname[r]);
            gUiStatus.rosterPing[r] = rping[r];
        }
    }

    // Pick up a start/stop request from the Qt thread. Runs before LAN-follow, so online wins.
    void pollRequest()
    {
        UiRequest r;
        {
            std::lock_guard<std::mutex> lk(gUiMx);
            if (!gUiReq.want) return;
            r = gUiReq;
            gUiReq.want = 0;
        }
        shutdown();
        started = true;
        if (r.want == 1) startHostOnline(r.addr, r.disp, r.name);
        else if (r.want == 2) startJoinOnline(r.addr, r.disp, r.code, r.name);
        else
        {
            setMsg("");
            printf("[BR] online: session stopped\n");
            fflush(stdout);
        }
        publishStatus();
    }

    void tick(melonDS::u32 frame)
    {
        if (mode == 0) return;
        if (online) onlineTick();
        else if (mode == 1 && listener != INVALID_SOCKET)
        {
            for (int i = 0; i < 7; i++)
            {
                if (peers[i].up) continue;
                SOCKET s = accept(listener, NULL, NULL);
                if (s == INVALID_SOCKET) break;
                setNonBlock(s);
                peers[i].s = s; peers[i].up = true; freshPeer = true;
                melonDS::u8 ctl[4] = { 2, 0, 0xFF, (melonDS::u8)(2 + i) };
                peers[i].tx.insert(peers[i].tx.end(), ctl, ctl + 4);
                printf("[BR] peer accepted -> role %d\n", 2 + i);
            }
        }
        else if (mode == 2 && !peers[0].up)
        {
            Peer& h = peers[0];
            if (connecting)
            {
                fd_set wr, ex; FD_ZERO(&wr); FD_ZERO(&ex);
                FD_SET(h.s, &wr); FD_SET(h.s, &ex);
                timeval tv = {0,0};
                int r = select(0, NULL, &wr, &ex, &tv);
                if (r > 0 && FD_ISSET(h.s, &wr)) { h.up = true; connecting = false; freshPeer = true; printf("[BR] connected to %s\n", joinIP); }
                else if (r > 0 && FD_ISSET(h.s, &ex)) { dropPeer(0); retryAt = frame + 120; }
            }
            else if (frame >= retryAt)
            {
                h.s = socket(AF_INET, SOCK_STREAM, 0);
                if (h.s == INVALID_SOCKET) { retryAt = frame + 120; return; }
                setNonBlock(h.s);
                sockaddr_in a; memset(&a, 0, sizeof(a));
                a.sin_family = AF_INET; a.sin_port = htons(PORT);
                inet_pton(AF_INET, joinIP, &a.sin_addr);
                int r = ::connect(h.s, (sockaddr*)&a, sizeof(a));
                if (r == 0) { h.up = true; freshPeer = true; printf("[BR] connected to %s\n", joinIP); }
                else if (WSAGetLastError() == WSAEWOULDBLOCK) connecting = true;
                else { dropPeer(0); retryAt = frame + 120; }
            }
        }
        for (int i = 0; i < 7; i++) { flush(i); pumpRecv(i); }
        lobbyTick(frame);
    }

    void enqueue(int i, const melonDS::u8* p, melonDS::u32 n)
    {
        Peer& pr = peers[i];
        if (!pr.up || n == 0 || n > MAXFRAME) return;
        if (pr.tx.size() > 512*1024) return;
        melonDS::u8 hdr[2] = { (melonDS::u8)(n & 0xFF), (melonDS::u8)(n >> 8) };
        pr.tx.insert(pr.tx.end(), hdr, hdr + 2);
        pr.tx.insert(pr.tx.end(), p, p + n);
        flush(i);
    }
    void sendAll(const melonDS::u8* p, melonDS::u32 n) { for (int i=0;i<7;i++) enqueue(i, p, n); }

    void flush(int i)
    {
        Peer& p = peers[i];
        if (!p.up || p.tx.empty()) return;
        int r = ::send(p.s, (const char*)p.tx.data(), (int)p.tx.size(), 0);
        if (r > 0) p.tx.erase(p.tx.begin(), p.tx.begin() + r);
        else if (r == SOCKET_ERROR && WSAGetLastError() != WSAEWOULDBLOCK) dropPeer(i);
    }

    void pumpRecv(int i)
    {
        Peer& p = peers[i];
        if (!p.up) return;
        char tmp[16384];
        for (;;)
        {
            int r = ::recv(p.s, tmp, sizeof(tmp), 0);
            if (r > 0) { p.rx.insert(p.rx.end(), tmp, tmp + r); if (p.rx.size() > 1024*1024) { dropPeer(i); return; } }
            else if (r == 0) { dropPeer(i); return; }
            else { if (WSAGetLastError() != WSAEWOULDBLOCK) dropPeer(i); return; }
        }
    }

    melonDS::u32 recvFrame(int i, melonDS::u8* out, melonDS::u32 outMax)
    {
        Peer& p = peers[i];
        if (p.rx.size() < 2) return 0;
        melonDS::u32 n = (melonDS::u32)p.rx[0] | ((melonDS::u32)p.rx[1] << 8);
        if (n == 0 || n > MAXFRAME) { dropPeer(i); return 0; }
        if (p.rx.size() < 2 + n) return 0;
        melonDS::u32 c = (n <= outMax) ? n : outMax;
        memcpy(out, p.rx.data() + 2, c);
        p.rx.erase(p.rx.begin(), p.rx.begin() + 2 + n);
        return c;
    }

    void shutdown()
    {
        online = 0;                 // before dropPeer: no "redial the room" here
        for (int i=0;i<7;i++) dropPeer(i);
        if (beaconTx != INVALID_SOCKET) { closesocket(beaconTx); beaconTx = INVALID_SOCKET; }
        if (listener != INVALID_SOCKET) { closesocket(listener); listener = INVALID_SOCKET; }
        ctl.close(); jlink.close();
        for (int i=0;i<7;i++) { acc[i].l.close(); acc[i].slot = -1; acc[i].ticket = 0; }
        roomCode[0] = 0; onlineFatal = false; onlineRetryMs = 0;
        assignedRole = 0;
        mode = 0;
    }
};
Net gNet;
}

struct BridgeSt
{
    melonDS::u32 frame = 0;
    melonDS::u32 disc = 0, ctl = 0;
    melonDS::u32 exportBlk = 0, importBlk = 0, partyExp = 0, partyImp = 0;
    melonDS::u32 pktExp = 0, pktImp = 0, owExp = 0, owImp = 0;
    melonDS::u32 blkN = 0, partyN = 0;
    melonDS::u32 blkSize = 0, partySize = 0, pktSize = 0;
    melonDS::u8 beat = 0;
    std::vector<melonDS::u8> lastParty, lastPkt;
    melonDS::u32 roleSeenAt[9] = {0};
    melonDS::u32 dbgTx = 0, dbgRx = 0, dbgPktTx = 0, dbgPktRx = 0;

    melonDS::u8 FreshPeerMask(int myRole) const
    {
        melonDS::u8 m = 0;
        for (int r = 1; r <= 8; r++)
            if (r != myRole && roleSeenAt[r] != 0 && frame - roleSeenAt[r] <= 180)
                m |= (melonDS::u8)(1 << (r - 1));
        return m;
    }
};
BridgeSt gBr;

void apWr8(melonDS::NDS* nds, melonDS::u32 addr, melonDS::u8 v)
{
    nds->MainRAM[addr & nds->MainRAMMask] = v;
}
melonDS::u8* apPtr(melonDS::NDS* nds, melonDS::u32 addr)
{
    return &nds->MainRAM[addr & nds->MainRAMMask];
}

void BridgePump(melonDS::NDS* nds)
{
    using namespace melonDS;
    gBr.frame++;

    mpnet::gNet.pollRequest();      // Host/Join Online Game... from the menu
    if (mpnet::gNet.mode == 0)
    {
        // No env config: follow the melonDS LAN lobby (host starts the bridge host, joiner connects to the lobby host).
        if (MPInterface::GetType() == MPInterface_LAN)
        {
            LAN& lan = (LAN&)MPInterface::Get();
            if (lan.GetIsHost())
            {
                mpnet::gNet.startHost();
            }
            else if (lan.GetMyPlayerID() > 0)
            {
                melonDS::u32 ha = lan.GetHostAddress();
                if (ha)
                {
                    char ip[32];
                    snprintf(ip, sizeof(ip), "%u.%u.%u.%u",
                        ha & 0xFF, (ha >> 8) & 0xFF, (ha >> 16) & 0xFF, (ha >> 24) & 0xFF);
                    mpnet::gNet.startJoin(ip);
                }
            }
        }
        if (mpnet::gNet.mode == 0) return;
    }
    mpnet::gNet.tick(gBr.frame);

    // A peer link just came up: clear the send caches so party/pkt are resent.
    if (mpnet::gNet.freshPeer)
    {
        mpnet::gNet.freshPeer = false;
        gBr.lastParty.clear();
        gBr.lastPkt.clear();
    }

    if (!gBr.disc)
    {
        if ((gBr.frame % 60) != 0) return;
        for (u32 off = 0; off < 0x400000 - 16; off += 4)
        {
            if (*(u32*)&nds->MainRAM[off] == 0xCAFE1234
                && *(u32*)&nds->MainRAM[off+4] == 0x5678CAFE)
            {
                gBr.disc = 0x02000000 + off;
                break;
            }
        }
        if (!gBr.disc) return;

        gBr.exportBlk = apRd32(nds, gBr.disc + 2*4);
        gBr.importBlk = apRd32(nds, gBr.disc + 3*4);
        gBr.partyExp  = apRd32(nds, gBr.disc + 4*4);
        gBr.partyImp  = apRd32(nds, gBr.disc + 5*4);
        gBr.pktExp    = apRd32(nds, gBr.disc + 12*4);
        gBr.pktImp    = apRd32(nds, gBr.disc + 13*4);
        gBr.owExp     = apRd32(nds, gBr.disc + 14*4);
        gBr.owImp     = apRd32(nds, gBr.disc + 15*4);
        gBr.blkSize   = apRd32(nds, gBr.disc + 20*4) & 0xFFFF;
        gBr.blkN      = apRd32(nds, gBr.disc + 25*4);
        gBr.partyN    = apRd32(nds, gBr.disc + 26*4);
        gBr.ctl       = apRd32(nds, gBr.disc + 31*4);
    }
    if (!gBr.ctl || apRd32(nds, gBr.ctl) != 0x42524731) return;

    gBr.partySize = apRd16s(nds, gBr.ctl + 10) & 0xFFFF;
    gBr.pktSize   = apRd16s(nds, gBr.ctl + 12) & 0xFFFF;

    apWr8(nds, gBr.ctl + 6, ++gBr.beat);   // fork heartbeat

    u8 wanted = apRd8(nds, gBr.ctl + 4);
    bool inGame = (wanted && gBr.blkSize != 0 && gBr.blkSize <= 512
        && gBr.partySize != 0 && gBr.partySize <= 2048
        && gBr.pktSize != 0 && gBr.pktSize <= 2048);

    // No early return when not in game: frames must still be drained and applied.
    if (!inGame)
    {
        apWr8(nds, gBr.ctl + 7, 0);
        apWr8(nds, gBr.ctl + 9, 0);
    }

    int myRole = mpnet::gNet.myRole();
    apWr8(nds, gBr.ctl + 8, (u8)myRole);

    if (inGame && mpnet::gNet.anyUp())
    {
        u8 buf[2100];

        // bundle: [1][role][blkSize u16][block][ow 48] — every frame
        buf[0] = 1; buf[1] = (u8)myRole;
        buf[2] = (u8)(gBr.blkSize & 0xFF); buf[3] = (u8)(gBr.blkSize >> 8);
        memcpy(buf + 4, apPtr(nds, gBr.exportBlk), gBr.blkSize);
        memcpy(buf + 4 + gBr.blkSize, apPtr(nds, gBr.owExp), 48);
        mpnet::gNet.sendAll(buf, 4 + gBr.blkSize + 48); gBr.dbgTx++;

        // party: on content change
        if (gBr.lastParty.size() != gBr.partySize
            || memcmp(gBr.lastParty.data(), apPtr(nds, gBr.partyExp), gBr.partySize) != 0)
        {
            gBr.lastParty.assign(apPtr(nds, gBr.partyExp), apPtr(nds, gBr.partyExp) + gBr.partySize);
            buf[0] = 2; buf[1] = (u8)myRole;
            buf[2] = (u8)(gBr.partySize & 0xFF); buf[3] = (u8)(gBr.partySize >> 8);
            memcpy(buf + 4, gBr.lastParty.data(), gBr.partySize);
            mpnet::gNet.sendAll(buf, 4 + gBr.partySize);
        }

        // pkt channel: on content change
        if (gBr.lastPkt.size() != gBr.pktSize
            || memcmp(gBr.lastPkt.data(), apPtr(nds, gBr.pktExp), gBr.pktSize) != 0)
        {
            gBr.lastPkt.assign(apPtr(nds, gBr.pktExp), apPtr(nds, gBr.pktExp) + gBr.pktSize);
            buf[0] = 3; buf[1] = (u8)myRole;
            buf[2] = (u8)(gBr.pktSize & 0xFF); buf[3] = (u8)(gBr.pktSize >> 8);
            memcpy(buf + 4, gBr.lastPkt.data(), gBr.pktSize);
            mpnet::gNet.sendAll(buf, 4 + gBr.pktSize); gBr.dbgPktTx++;
        }
    }

    // receive: apply peers' mailboxes (per peer link; host relays)
    {
        u8 rx[2100];
        u32 n;
        static melonDS::u8 sGameEver[9] = {0};   // ever game-active latch (strictness)
        for (int pi = 0; pi < 7; pi++)
        while ((n = mpnet::gNet.recvFrame(pi, rx, sizeof(rx))) > 0)
        {
            gBr.dbgRx++;
            // Take control frames out before the bundle path.
            if (mpnet::gNet.handleCtlFrame(pi, rx, n, myRole)) continue;
            if (n < 4) continue;
            int tag = rx[0], r = rx[1];
            u32 sz = (u32)(rx[2] | (rx[3] << 8));
            if (r < 1 || r > 8 || r == myRole) continue;
            if (!mpnet::gNet.roleAllowed(pi, r)) continue;
            if (n < 4 + sz) continue;

            // Host relay: forward each client's bundle to the other clients.
            if (myRole == 1)
                for (int pj = 0; pj < 7; pj++)
                    if (pj != pi) mpnet::gNet.enqueue(pj, rx, n);

            if (tag >= 1 && tag <= 3)
            {
                // Peer newly in game: resend our party/pkt.
                bool wasFresh = gBr.roleSeenAt[r] != 0 && gBr.frame - gBr.roleSeenAt[r] <= 180;
                if (!wasFresh)
                {
                    gBr.lastParty.clear();
                    gBr.lastPkt.clear();
                }
                // game presence only: lobby idlers do not count
                gBr.roleSeenAt[r] = gBr.frame;
                sGameEver[r] = 1;   // latch: role has been game-active this session
            }

            // The pairwise import only takes the chosen partner (pairRole at OW export +0x18).
            if (tag == 1 && sz == gBr.blkSize && n >= 4 + sz + 48)
            {
                // With 2+ game peers the pairwise import is strict: only the chosen partner, never "first come".
                melonDS::u8 pairRole = apRd8(nds, gBr.owExp + 0x18);
                // count peers that were ever in game, not recent traffic
                int gamePeers = 0;
                for (int gr = 1; gr <= 8; gr++)
                    if (gr != myRole && sGameEver[gr]) gamePeers++;
                bool strict = (gamePeers >= 2);
                rx[4 + 0x12] = (melonDS::u8)r;   // stamp the sender's role into the block
                if ((pairRole == 0 && !strict) || r == (int)pairRole)
                    memcpy(apPtr(nds, gBr.importBlk), rx + 4, sz);
                if (gBr.blkN) memcpy(apPtr(nds, gBr.blkN + (r-1)*sz), rx + 4, sz);
                memcpy(apPtr(nds, gBr.owImp + (r-1)*48), rx + 4 + sz, 48);
            }
            else if (tag == 2 && sz == gBr.partySize)
            {
                melonDS::u8 pairRole = apRd8(nds, gBr.owExp + 0x18);
                // count peers that were ever in game, not recent traffic
                int gamePeers = 0;
                for (int gr = 1; gr <= 8; gr++)
                    if (gr != myRole && sGameEver[gr]) gamePeers++;
                bool strict = (gamePeers >= 2);
                if ((pairRole == 0 && !strict) || r == (int)pairRole)
                    memcpy(apPtr(nds, gBr.partyImp), rx + 4, sz);
                if (gBr.partyN) memcpy(apPtr(nds, gBr.partyN + (r-1)*sz), rx + 4, sz);
            }
            else if (tag == 3 && sz == gBr.pktSize)
            {
                memcpy(apPtr(nds, gBr.pktImp + (r-1)*sz), rx + 4, sz);
                gBr.dbgPktRx++;
            }
        }
    }

    // pairRole changed to a real role: replay that role's cached block and party. A change to 0 is ignored (it flaps).
    if (gBr.owExp && gBr.importBlk)
    {
        static melonDS::u8 sLastPairRole = 0xFF;
        melonDS::u8 pr = apRd8(nds, gBr.owExp + 0x18);
        if (pr != sLastPairRole)
        {
            sLastPairRole = pr;
            if (pr >= 1 && pr <= 8)
            {
                if (gBr.blkN)
                    memcpy(apPtr(nds, gBr.importBlk), apPtr(nds, gBr.blkN + (pr-1)*gBr.blkSize), gBr.blkSize);
                if (gBr.partyN && gBr.partyImp)
                    memcpy(apPtr(nds, gBr.partyImp), apPtr(nds, gBr.partyN + (pr-1)*gBr.partySize), gBr.partySize);
            }
        }
    }
    if (gBr.exportBlk && gBr.importBlk)
    {
        static melonDS::u8 sLastOwnInBattle = 0;
        melonDS::u8 ib = apRd8(nds, gBr.exportBlk + 0x10);
        if (!ib && sLastOwnInBattle)
            apWr32(nds, gBr.importBlk, 0);   // battle over: purge partner-block ghost
        sLastOwnInBattle = ib;
    }

    if ((gBr.frame % 120) == 0)
    {
        {
            melonDS::u32 shim = apRd32(nds, gBr.disc + 16*4);
            melonDS::u8 shimA = shim ? apRd8(nds, shim) : 0xFF;
            melonDS::u32 commTask = shim ? apRd32(nds, shim + 12) : 0;
            printf("[BR] f=%u role=%d wanted=%u tx=%u rx=%u pktTx=%u pktRx=%u shim=%u commTask=%08X%c",
                gBr.frame, myRole, wanted, gBr.dbgTx, gBr.dbgRx,
                gBr.dbgPktTx, gBr.dbgPktRx, shimA, commTask, 10);
        }
        fflush(stdout);
    }

    // Crash catcher: overworld frame counter frozen during a session, so dump the ARM9 registers.
    {
        static melonDS::u32 ccLastFC = 0, ccChangedAt = 0;
        static int ccDumped = 0;
        melonDS::u32 fcNow = apRd32(nds, gBr.owExp + 0x0C);
        if (fcNow != ccLastFC)
        {
            ccLastFC = fcNow;
            ccChangedAt = gBr.frame;
            ccDumped = 0;
        }
        else if (wanted && gBr.frame - ccChangedAt > 180 && ccDumped < 24)
        {
            FILE* cf = fopen("melonds_hangpc_auto.txt", "a");
            if (cf)
            {
                fprintf(cf, "role=%d f=%u FCstuck=%u R15=%08X R14=%08X R13=%08X R12=%08X CPSR=%08X%c",
                    myRole, gBr.frame, fcNow,
                    nds->ARM9.R[15], nds->ARM9.R[14], nds->ARM9.R[13], nds->ARM9.R[12],
                    nds->ARM9.CPSR, 10);
                fprintf(cf, "  regs R0-11:");
                for (int ri = 0; ri < 12; ri++) fprintf(cf, " %08X", nds->ARM9.R[ri]);
                fprintf(cf, "%c  banked SVC=%08X,%08X ABT=%08X,%08X IRQ=%08X,%08X%c",
                    10, nds->ARM9.R_SVC[0], nds->ARM9.R_SVC[1],
                    nds->ARM9.R_ABT[0], nds->ARM9.R_ABT[1],
                    nds->ARM9.R_IRQ[0], nds->ARM9.R_IRQ[1], 10);
                // stack dump from the most plausible pre-crash SP
                melonDS::u32 sp = nds->ARM9.R_ABT[0] ? nds->ARM9.R_ABT[0] : nds->ARM9.R_SVC[0];
                if (!sp || (sp >> 24) != 0x02) sp = nds->ARM9.R_SVC[0];
                if (sp && (sp >> 24) == 0x02)
                {
                    fprintf(cf, "  stack@%08X:", sp);
                    for (int si = 0; si < 32; si++)
                        fprintf(cf, " %08X", apRd32(nds, sp + si*4));
                    fprintf(cf, "%c", 10);
                }
                fclose(cf);
                ccDumped++;
            }
        }
    }

    // Status + peer mask, only while our session is active.
    if (inGame)
    {
        u8 mask = gBr.FreshPeerMask(myRole);
        apWr8(nds, gBr.ctl + 9, mask);
        apWr8(nds, gBr.ctl + 7, mask ? 2 : 1);
        mpnet::gNet.beaconTick(gBr.frame, mask ? 2 : 1);
    }
}

}
// ---------------------------------------------------------------------------


// Online relay control surface (MpOnline.h): called from the Qt thread, served by the next bridge pump.

int MpOnlineWireVersion() { return (int)mpnet::WIREVER; }

// Blocking DNS: runs once at click time or env init, never per frame.
bool MpOnlineResolve(const char* server, int defPort, MpOnlineTarget* out)
{
    if (!server || !out) return false;
    while (*server == ' ') server++;
    if (!*server) return false;

    char host[128];
    mpnet::setStr(host, server);
    for (int i = (int)strlen(host) - 1; i >= 0 && host[i] == ' '; i--) host[i] = 0;

    int port = defPort;
    char* colon = strrchr(host, ':');
    if (colon && colon[1])
    {
        int p = atoi(colon + 1);
        if (p > 0 && p < 65536) { port = p; *colon = 0; }
    }
    if (!host[0]) return false;

    WSADATA w; WSAStartup(MAKEWORD(2,2), &w);

    addrinfo hints = {};
    hints.ai_family = AF_INET;              // the bridge speaks IPv4
    hints.ai_socktype = SOCK_STREAM;
    char portstr[16];
    snprintf(portstr, sizeof(portstr), "%d", port);
    addrinfo* res = nullptr;
    if (getaddrinfo(host, portstr, &hints, &res) != 0 || !res) return false;

    sockaddr_in a = {};
    memcpy(&a, res->ai_addr, sizeof(a));
    freeaddrinfo(res);

    memcpy(out->addr, &a, sizeof(a));
    out->addrLen = (int)sizeof(a);
    snprintf(out->disp, sizeof(out->disp), "%s:%d", host, port);
    return true;
}

static void MpOnlinePost(int want, const MpOnlineTarget& t, const char* code, const char* name)
{
    std::lock_guard<std::mutex> lk(mpnet::gUiMx);
    mpnet::gUiReq.want = want;
    memcpy(&mpnet::gUiReq.addr, t.addr, sizeof(mpnet::gUiReq.addr));
    mpnet::setStr(mpnet::gUiReq.disp, t.disp);
    mpnet::setStr(mpnet::gUiReq.name, name);
    mpnet::setStr(mpnet::gUiReq.code, code);
    mpnet::gUiStatus.pending = true;
    mpnet::gUiStatus.mode = (want == 3) ? 0 : want;
    mpnet::setStr(mpnet::gUiStatus.server, t.disp);
    mpnet::setStr(mpnet::gUiStatus.code, (want == 2) ? code : "");
    mpnet::setStr(mpnet::gUiStatus.text,
        (want == 3) ? "" : "starting (waiting for emulation)");
}

void MpOnlineHost(const MpOnlineTarget& t, const char* name)
{
    MpOnlinePost(1, t, "", name);
}

void MpOnlineJoin(const MpOnlineTarget& t, const char* code, const char* name)
{
    MpOnlinePost(2, t, code, name);
}

void MpOnlineStop()
{
    MpOnlineTarget t;
    MpOnlinePost(3, t, "", "");
}

void MpOnlineGetStatus(MpOnlineStatus* out)
{
    if (!out) return;
    std::lock_guard<std::mutex> lk(mpnet::gUiMx);
    *out = mpnet::gUiStatus;
}


void EmuThread::run()
{
    Config::Table& globalCfg = emuInstance->getGlobalConfig();
    u32 mainScreenPos[3];

    //emuInstance->updateConsole();
    // No carts are inserted when melonDS first boots

    mainScreenPos[0] = 0;
    mainScreenPos[1] = 0;
    mainScreenPos[2] = 0;
    autoScreenSizing = 0;

    //videoSettingsDirty = false;

    if (emuInstance->usesOpenGL())
    {
        emuInstance->initOpenGL(0);

        useOpenGL = true;
        videoRenderer = globalCfg.GetInt("3D.Renderer");
    }
    else
    {
        useOpenGL = false;
        videoRenderer = 0;
    }

    //updateRenderer();
    videoSettingsDirty = true;

    u32 nframes = 0;
    double perfCountsSec = 1.0 / SDL_GetPerformanceFrequency();
    double lastTime = SDL_GetPerformanceCounter() * perfCountsSec;
    double frameLimitError = 0.0;
    double lastMeasureTime = lastTime;

    u32 winUpdateCount = 0, winUpdateFreq = 1;
    u8 dsiVolumeLevel = 0x1F;

    char melontitle[100];

    bool fastforward = false;
    bool slowmo = false;
    emuInstance->fastForwardToggled = false;
    emuInstance->slowmoToggled = false;

    while (emuStatus != emuStatus_Exit)
    {
        if (emuInstance->instanceID == 0)
            MPInterface::Get().Process();

        emuInstance->inputProcess();

        if (emuInstance->hotkeyPressed(HK_FrameLimitToggle)) emit windowLimitFPSChange();

        if (emuInstance->hotkeyPressed(HK_Pause)) emuTogglePause();
        if (emuInstance->hotkeyPressed(HK_Reset)) emuReset();
        if (emuInstance->hotkeyPressed(HK_FrameStep)) emuFrameStep();

        if (emuInstance->hotkeyPressed(HK_FullscreenToggle)) emit windowFullscreenToggle();

        if (emuInstance->hotkeyPressed(HK_SwapScreens)) emit swapScreensToggle();
        if (emuInstance->hotkeyPressed(HK_SwapScreenEmphasis)) emit screenEmphasisToggle();

        if (emuStatus == emuStatus_Running || emuStatus == emuStatus_FrameStep)
        {
            if (emuStatus == emuStatus_FrameStep) emuStatus = emuStatus_Paused;

            if (emuInstance->hotkeyPressed(HK_SolarSensorDecrease))
            {
                int level = emuInstance->nds->GBACartSlot.SetInput(GBACart::Input_SolarSensorDown, true);
                if (level != -1)
                {
                    emuInstance->osdAddMessage(0, "Solar sensor level: %d", level);
                }
            }
            if (emuInstance->hotkeyPressed(HK_SolarSensorIncrease))
            {
                int level = emuInstance->nds->GBACartSlot.SetInput(GBACart::Input_SolarSensorUp, true);
                if (level != -1)
                {
                    emuInstance->osdAddMessage(0, "Solar sensor level: %d", level);
                }
            }

            if (emuInstance->nds->ConsoleType == 1)
            {
                DSi* dsi = static_cast<DSi*>(emuInstance->nds);
                double currentTime = SDL_GetPerformanceCounter() * perfCountsSec;

                // Handle power button
                if (emuInstance->hotkeyDown(HK_PowerButton))
                {
                    dsi->I2C.GetBPTWL()->SetPowerButtonHeld(currentTime);
                }
                else if (emuInstance->hotkeyReleased(HK_PowerButton))
                {
                    dsi->I2C.GetBPTWL()->SetPowerButtonReleased(currentTime);
                }

                // Handle volume buttons
                if (emuInstance->hotkeyDown(HK_VolumeUp))
                {
                    dsi->I2C.GetBPTWL()->SetVolumeSwitchHeld(DSi_BPTWL::volumeKey_Up);
                }
                else if (emuInstance->hotkeyReleased(HK_VolumeUp))
                {
                    dsi->I2C.GetBPTWL()->SetVolumeSwitchReleased(DSi_BPTWL::volumeKey_Up);
                }

                if (emuInstance->hotkeyDown(HK_VolumeDown))
                {
                    dsi->I2C.GetBPTWL()->SetVolumeSwitchHeld(DSi_BPTWL::volumeKey_Down);
                }
                else if (emuInstance->hotkeyReleased(HK_VolumeDown))
                {
                    dsi->I2C.GetBPTWL()->SetVolumeSwitchReleased(DSi_BPTWL::volumeKey_Down);
                }

                dsi->I2C.GetBPTWL()->ProcessVolumeSwitchInput(currentTime);
            }

            if (useOpenGL)
                emuInstance->makeCurrentGL();

            // update render settings if needed
            if (videoSettingsDirty)
            {
                emuInstance->renderLock.lock();
                if (useOpenGL)
                {
                    emuInstance->setVSyncGL(true);
                    videoRenderer = globalCfg.GetInt("3D.Renderer");
                }
#ifdef OGLRENDERER_ENABLED
                else
#endif
                {
                    videoRenderer = 0;
                }

                updateRenderer();

                videoSettingsDirty = false;
                emuInstance->renderLock.unlock();
            }

            // process input and hotkeys
            BridgePump(emuInstance->nds);
            emuInstance->nds->SetKeyMask(emuInstance->inputMask);

            if (emuInstance->isTouching)
                emuInstance->nds->TouchScreen(emuInstance->touchX, emuInstance->touchY);
            else
                emuInstance->nds->ReleaseScreen();

            if (emuInstance->hotkeyPressed(HK_Lid))
            {
                bool lid = !emuInstance->nds->IsLidClosed();
                emuInstance->nds->SetLidClosed(lid);
                emuInstance->osdAddMessage(0, lid ? "Lid closed" : "Lid opened");
            }

            // auto screen layout
            {
                mainScreenPos[2] = mainScreenPos[1];
                mainScreenPos[1] = mainScreenPos[0];
                mainScreenPos[0] = emuInstance->nds->PowerControl9 >> 15;

                int guess;
                if (mainScreenPos[0] == mainScreenPos[2] &&
                    mainScreenPos[0] != mainScreenPos[1])
                {
                    // constant flickering, likely displaying 3D on both screens
                    // TODO: when both screens are used for 2D only...???
                    guess = screenSizing_Even;
                }
                else
                {
                    if (mainScreenPos[0] == 1)
                        guess = screenSizing_EmphTop;
                    else
                        guess = screenSizing_EmphBot;
                }

                if (guess != autoScreenSizing)
                {
                    autoScreenSizing = guess;
                    emit autoScreenSizingChange(autoScreenSizing);
                }
            }

            // RTC sync
            emuInstance->syncRTC();


            // emulate
            u32 nlines;
            if (emuInstance->nds->GPU.GetRenderer().NeedsShaderCompile())
            {
                compileShaders();
                nlines = 1;
            }
            else
            {
                nlines = emuInstance->nds->RunFrame();
            }

            if (emuInstance->ndsSave)
                emuInstance->ndsSave->CheckFlush();

            if (emuInstance->gbaSave)
                emuInstance->gbaSave->CheckFlush();

            if (emuInstance->firmwareSave)
                emuInstance->firmwareSave->CheckFlush();

            emuInstance->drawScreen();

#ifdef MELONCAP
            MelonCap::Update();
#endif // MELONCAP

            winUpdateCount++;
            if (winUpdateCount >= winUpdateFreq && !useOpenGL)
            {
                emit windowUpdate();
                winUpdateCount = 0;
            }
            
            if (emuInstance->hotkeyPressed(HK_FastForwardToggle)) emuInstance->fastForwardToggled = !emuInstance->fastForwardToggled;
            if (emuInstance->hotkeyPressed(HK_SlowMoToggle)) emuInstance->slowmoToggled = !emuInstance->slowmoToggled;

            if (emuInstance->hotkeyPressed(HK_AudioMuteToggle)) emuInstance->toggleAudioMute();

            bool enablefastforward = emuInstance->hotkeyDown(HK_FastForward) | emuInstance->fastForwardToggled;
            bool enableslowmo = emuInstance->hotkeyDown(HK_SlowMo) | emuInstance->slowmoToggled;

            if (useOpenGL)
            {
                // when using OpenGL: when toggling fast-forward or slowmo, change the vsync interval
                if ((enablefastforward || enableslowmo) && !(fastforward || slowmo))
                {
                    emuInstance->setVSyncGL(false);
                }
                else if (!(enablefastforward || enableslowmo) && (fastforward || slowmo))
                {
                    emuInstance->setVSyncGL(true);
                }
            }

            fastforward = enablefastforward;
            slowmo = enableslowmo;
            emuInstance->updateFastForwardMute(fastforward);

            if (slowmo) emuInstance->curFPS = emuInstance->slowmoFPS;
            else if (fastforward) emuInstance->curFPS = emuInstance->fastForwardFPS;
            else if (!emuInstance->doLimitFPS && !emuInstance->doAudioSync) emuInstance->curFPS = 1000.0;
            else emuInstance->curFPS = emuInstance->targetFPS;

            if (emuInstance->audioDSiVolumeSync && emuInstance->nds->ConsoleType == 1)
            {
                DSi* dsi = static_cast<DSi*>(emuInstance->nds);
                u8 volumeLevel = dsi->I2C.GetBPTWL()->GetVolumeLevel();
                if (volumeLevel != dsiVolumeLevel)
                {
                    dsiVolumeLevel = volumeLevel;
                    emit syncVolumeLevel();
                }

                emuInstance->audioVolume = volumeLevel * (256.0 / 31.0);
            }

            if (emuInstance->doAudioSync && !(fastforward || slowmo))
                emuInstance->audioSync();

            double frametimeStep = nlines / (emuInstance->curFPS * 263.0);

            if (frametimeStep < 0.001) frametimeStep = 0.001;

            if (emuInstance->doLimitFPS)
            {
                double curtime = SDL_GetPerformanceCounter() * perfCountsSec;

                frameLimitError += frametimeStep - (curtime - lastTime);
                if (frameLimitError < -frametimeStep)
                    frameLimitError = -frametimeStep;
                if (frameLimitError > frametimeStep)
                    frameLimitError = frametimeStep;

                if (round(frameLimitError * 1000.0) > 0.0)
                {
                    SDL_Delay(round(frameLimitError * 1000.0));
                    double timeBeforeSleep = curtime;
                    curtime = SDL_GetPerformanceCounter() * perfCountsSec;
                    frameLimitError -= curtime - timeBeforeSleep;
                }

                lastTime = curtime;
            }

            nframes++;
            if (nframes >= 30)
            {
                double time = SDL_GetPerformanceCounter() * perfCountsSec;
                double dt = time - lastMeasureTime;
                lastMeasureTime = time;

                u32 fps = round(nframes / dt);
                nframes = 0;

                float fpstarget = 1.0/frametimeStep;

                winUpdateFreq = fps / (u32)round(fpstarget);
                if (winUpdateFreq < 1)
                    winUpdateFreq = 1;
                    
                double actualfps = (59.8261 * 263.0) / nlines;
                snprintf(melontitle, sizeof(melontitle), "[%d/%.0f] melonDS " MELONDS_VERSION, fps, actualfps);
                changeWindowTitle(melontitle);
            }
        }
        else
        {
            // paused
            nframes = 0;
            lastTime = SDL_GetPerformanceCounter() * perfCountsSec;
            lastMeasureTime = lastTime;

            emit windowUpdate();

            snprintf(melontitle, sizeof(melontitle), "melonDS " MELONDS_VERSION);
            changeWindowTitle(melontitle);

            SDL_Delay(75);

            emuInstance->drawScreen();
        }

        handleMessages();
    }
}

void EmuThread::sendMessage(Message msg)
{
    msgMutex.lock();
    msgQueue.enqueue(msg);
    msgMutex.unlock();
}

void EmuThread::waitMessage(int num)
{
    if (QThread::currentThread() == this) return;
    msgSemaphore.acquire(num);
}

void EmuThread::waitAllMessages()
{
    if (QThread::currentThread() == this) return;
    while (!msgQueue.empty())
        msgSemaphore.acquire();
}

void EmuThread::handleMessages()
{
    bool glborrow = false;

    msgMutex.lock();
    while (!msgQueue.empty())
    {
        Message msg = msgQueue.dequeue();
        switch (msg.type)
        {
        case msg_Exit:
            emuStatus = emuStatus_Exit;
            emuPauseStack = emuPauseStackRunning;

            emuInstance->audioDisable();
            MPInterface::Get().End(emuInstance->instanceID);
            break;

        case msg_EmuRun:
            emuStatus = emuStatus_Running;
            emuPauseStack = emuPauseStackRunning;
            emuActive = true;

            emuInstance->audioEnable();
            emit windowEmuStart();
            break;

        case msg_EmuPause:
            emuPauseStack++;
            if (emuPauseStack > emuPauseStackPauseThreshold) break;

            prevEmuStatus = emuStatus;
            emuStatus = emuStatus_Paused;

            if (prevEmuStatus != emuStatus_Paused)
            {
                emuInstance->audioDisable();
                emit windowEmuPause(true);
                emuInstance->osdAddMessage(0, "Paused");
            }
            break;

        case msg_EmuUnpause:
            if (emuPauseStack < emuPauseStackPauseThreshold) break;

            emuPauseStack--;
            if (emuPauseStack >= emuPauseStackPauseThreshold) break;

            emuStatus = prevEmuStatus;

            if (emuStatus != emuStatus_Paused)
            {
                emuInstance->audioEnable();
                emit windowEmuPause(false);
                emuInstance->osdAddMessage(0, "Resumed");
            }
            break;

        case msg_EmuStop:
            if (msg.param.value<bool>())
                emuInstance->nds->Stop();
            emuStatus = emuStatus_Paused;
            emuActive = false;

            emuInstance->audioDisable();
            emit windowEmuStop();
            break;

        case msg_EmuFrameStep:
            emuStatus = emuStatus_FrameStep;
            break;

        case msg_EmuReset:
            emuInstance->reset();

            emuStatus = emuStatus_Running;
            emuPauseStack = emuPauseStackRunning;
            emuActive = true;

            emuInstance->audioEnable();
            emit windowEmuReset();
            emuInstance->osdAddMessage(0, "Reset");
            break;

        case msg_InitGL:
            emuInstance->initOpenGL(msg.param.value<int>());
            useOpenGL = true;
            break;

        case msg_DeInitGL:
            emuInstance->deinitOpenGL(msg.param.value<int>());
            if (msg.param.value<int>() == 0)
                useOpenGL = false;
            break;

        case msg_BorrowGL:
            emuInstance->releaseGL();
            glborrow = true;
            break;

        case msg_BootROM:
            msgResult = 0;
            if (!emuInstance->loadROM(msg.param.value<QStringList>(), true, msgError))
                break;

            assert(emuInstance->nds != nullptr);
            emuInstance->nds->Start();
            msgResult = 1;
            break;

        case msg_BootFirmware:
            msgResult = 0;
            if (!emuInstance->bootToMenu(msgError))
                break;

            assert(emuInstance->nds != nullptr);
            emuInstance->nds->Start();
            msgResult = 1;
            break;

        case msg_InsertCart:
            msgResult = 0;
            if (!emuInstance->loadROM(msg.param.value<QStringList>(), false, msgError))
                break;

            msgResult = 1;
            break;

        case msg_EjectCart:
            emuInstance->ejectCart();
            break;

        case msg_InsertGBACart:
            msgResult = 0;
            if (!emuInstance->loadGBAROM(msg.param.value<QStringList>(), msgError))
                break;

            msgResult = 1;
            break;

        case msg_InsertGBAAddon:
            msgResult = 0;
            emuInstance->loadGBAAddon(msg.param.value<int>(), msgError);
            msgResult = 1;
            break;

        case msg_EjectGBACart:
            emuInstance->ejectGBACart();
            break;

        case msg_SaveState:
            msgResult = emuInstance->saveState(msg.param.value<QString>().toStdString());
            break;

        case msg_LoadState:
            msgResult = emuInstance->loadState(msg.param.value<QString>().toStdString());
            break;

        case msg_UndoStateLoad:
            emuInstance->undoStateLoad();
            msgResult = 1;
            break;

        case msg_ImportSavefile:
            {
                msgResult = 0;
                auto f = Platform::OpenFile(msg.param.value<QString>().toStdString(), Platform::FileMode::Read);
                if (!f) break;

                u32 len = FileLength(f);

                std::unique_ptr<u8[]> data = std::make_unique<u8[]>(len);
                Platform::FileRewind(f);
                Platform::FileRead(data.get(), len, 1, f);

                assert(emuInstance->nds != nullptr);
                emuInstance->nds->SetNDSSave(data.get(), len);

                CloseFile(f);
                msgResult = 1;
            }
            break;

        case msg_EnableCheats:
            emuInstance->enableCheats(msg.param.value<bool>());
            break;
        }

        msgSemaphore.release();
    }
    msgMutex.unlock();

    if (glborrow)
    {
        glBorrowMutex.lock();
        glBorrowCond.wait(&glBorrowMutex);
        glBorrowMutex.unlock();
    }
}

void EmuThread::changeWindowTitle(char* title)
{
    emit windowTitleChange(QString(title));
}

void EmuThread::initContext(int win)
{
    sendMessage({.type = msg_InitGL, .param = win});
    waitMessage();
}

void EmuThread::deinitContext(int win)
{
    sendMessage({.type = msg_DeInitGL, .param = win});
    waitMessage();
}

void EmuThread::borrowGL()
{
    sendMessage(msg_BorrowGL);
    waitMessage();
}

void EmuThread::returnGL()
{
    glBorrowMutex.lock();
    glBorrowCond.wakeAll();
    glBorrowMutex.unlock();
}

void EmuThread::emuRun()
{
    sendMessage(msg_EmuRun);
    waitMessage();
}

void EmuThread::emuPause(bool broadcast)
{
    sendMessage(msg_EmuPause);
    waitMessage();

    if (broadcast)
        emuInstance->broadcastCommand(InstCmd_Pause);
}

void EmuThread::emuUnpause(bool broadcast)
{
    sendMessage(msg_EmuUnpause);
    waitMessage();

    if (broadcast)
        emuInstance->broadcastCommand(InstCmd_Unpause);
}

void EmuThread::emuTogglePause(bool broadcast)
{
    if (emuStatus == emuStatus_Paused)
        emuUnpause(broadcast);
    else
        emuPause(broadcast);
}

void EmuThread::emuStop(bool external)
{
    sendMessage({.type = msg_EmuStop, .param = external});
    waitMessage();
}

void EmuThread::emuExit()
{
    sendMessage(msg_Exit);
    waitAllMessages();
}

void EmuThread::emuFrameStep()
{
    if (emuPauseStack < emuPauseStackPauseThreshold)
        sendMessage(msg_EmuPause);
    sendMessage(msg_EmuFrameStep);
    waitAllMessages();
}

void EmuThread::emuReset()
{
    sendMessage(msg_EmuReset);
    waitMessage();
}

bool EmuThread::emuIsRunning()
{
    return emuStatus == emuStatus_Running;
}

bool EmuThread::emuIsActive()
{
    return emuActive;
}

int EmuThread::bootROM(const QStringList& filename, QString& errorstr)
{
    sendMessage({.type = msg_BootROM, .param = filename});
    waitMessage();
    if (!msgResult)
    {
        errorstr = msgError;
        return msgResult;
    }

    sendMessage(msg_EmuRun);
    waitMessage();
    errorstr = "";
    return msgResult;
}

int EmuThread::bootFirmware(QString& errorstr)
{
    sendMessage(msg_BootFirmware);
    waitMessage();
    if (!msgResult)
    {
        errorstr = msgError;
        return msgResult;
    }

    sendMessage(msg_EmuRun);
    waitMessage();
    errorstr = "";
    return msgResult;
}

int EmuThread::insertCart(const QStringList& filename, bool gba, QString& errorstr)
{
    MessageType msgtype = gba ? msg_InsertGBACart : msg_InsertCart;

    sendMessage({.type = msgtype, .param = filename});
    waitMessage();
    errorstr = msgResult ? "" : msgError;
    return msgResult;
}

void EmuThread::ejectCart(bool gba)
{
    sendMessage(gba ? msg_EjectGBACart : msg_EjectCart);
    waitMessage();
}

int EmuThread::insertGBAAddon(int type, QString& errorstr)
{
    sendMessage({.type = msg_InsertGBAAddon, .param = type});
    waitMessage();
    errorstr = msgResult ? "" : msgError;
    return msgResult;
}

int EmuThread::saveState(const QString& filename)
{
    sendMessage({.type = msg_SaveState, .param = filename});
    waitMessage();
    return msgResult;
}

int EmuThread::loadState(const QString& filename)
{
    sendMessage({.type = msg_LoadState, .param = filename});
    waitMessage();
    return msgResult;
}

int EmuThread::undoStateLoad()
{
    sendMessage(msg_UndoStateLoad);
    waitMessage();
    return msgResult;
}

int EmuThread::importSavefile(const QString& filename)
{
    sendMessage(msg_EmuReset);
    sendMessage({.type = msg_ImportSavefile, .param = filename});
    waitMessage(2);
    return msgResult;
}

void EmuThread::enableCheats(bool enable)
{
    sendMessage({.type = msg_EnableCheats, .param = enable});
    waitMessage();
}

void EmuThread::updateRenderer()
{
    auto nds = emuInstance->nds;

    if (videoRenderer != lastVideoRenderer)
    {
        switch (videoRenderer)
        {
            case renderer3D_Software:
                nds->SetRenderer(std::make_unique<SoftRenderer>(*nds));
                break;
            case renderer3D_OpenGL:
                nds->SetRenderer(std::make_unique<GLRenderer>(*nds, false));
                break;
            case renderer3D_OpenGLCompute:
                nds->SetRenderer(std::make_unique<GLRenderer>(*nds, true));
                break;
            default: __builtin_unreachable();
        }
    }
    lastVideoRenderer = videoRenderer;

    auto& cfg = emuInstance->getGlobalConfig();
    melonDS::RendererSettings settings = {
        .ScaleFactor = cfg.GetInt("3D.GL.ScaleFactor"),
        .Threaded = cfg.GetBool("3D.Soft.Threaded"),
        .HiresCoordinates = cfg.GetBool("3D.GL.HiresCoordinates"),
        .BetterPolygons = cfg.GetBool("3D.GL.BetterPolygons")
    };

    nds->GetRenderer().SetRenderSettings(settings);
}

void EmuThread::compileShaders()
{
    auto& renderer = emuInstance->nds->GPU.GetRenderer();
    int currentShader, shadersCount;
    u64 startTime = SDL_GetPerformanceCounter();
    // kind of hacky to look at the wallclock, though it is easier than
    // than disabling vsync
    do
    {
        renderer.ShaderCompileStep(currentShader, shadersCount);
    }
    while (renderer.NeedsShaderCompile() &&
             (SDL_GetPerformanceCounter() - startTime) * perfCountsSec < 1.0 / 6.0);
    emuInstance->osdAddMessage(0, "Compiling shader %d/%d", currentShader+1, shadersCount);
}
