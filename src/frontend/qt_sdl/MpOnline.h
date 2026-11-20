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

// Online relay (PMRELAY1) control surface for the Qt UI. No winsock here; the transport lives in EmuThread.cpp.

#ifndef MPONLINE_H
#define MPONLINE_H

// A relay server address resolved once, ahead of any frame work.
struct MpOnlineTarget
{
    unsigned char addr[32] = {};    // sockaddr_in, opaque to the UI
    int addrLen = 0;
    char disp[96] = "";             // "host:port" for display
};

struct MpOnlineStatus
{
    int mode = 0;                   // 0 off, 1 hosting online, 2 joined online
    int peers = 0;                  // live game links
    bool pending = false;           // request posted, emulation has not run yet
    char code[8] = "";              // room code (empty until the relay assigns)
    char server[96] = "";
    char text[128] = "";            // connection state / last error, for the UI

    // Lobby roster by role 1..8 (slot 0 unused).
    char roster[9][24] = {};
    int rosterPing[9] = {};         // that player's ping to the host, ms
    int myRole = 0;
};

// Resolves "host" or "host:port" (blocking).  false = could not resolve.
bool MpOnlineResolve(const char* server, int defPort, MpOnlineTarget* out);

// Post a start request.  Takes precedence over the native LAN session follow.
void MpOnlineHost(const MpOnlineTarget& t, const char* name);
void MpOnlineJoin(const MpOnlineTarget& t, const char* code, const char* name);
void MpOnlineStop();

void MpOnlineGetStatus(MpOnlineStatus* out);

// The bridge wire protocol version, as advertised by the LAN beacon.
int MpOnlineWireVersion();

// Default relay port when the player types a bare host name.
const int MpOnlineDefaultPort = 7833;

// Community relay pre-filled in the online dialogs; a saved custom value wins.
const char* const MpOnlineDefaultRelay = "193.122.236.144:7833";

#endif // MPONLINE_H
