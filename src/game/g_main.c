/*
===========================================================================

Wolfenstein: Enemy Territory GPL Source Code
Copyright (C) 1999-2010 id Software LLC, a ZeniMax Media company. 

This file is part of the Wolfenstein: Enemy Territory GPL Source Code (Wolf ET Source Code).  

Wolf ET Source Code is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Wolf ET Source Code is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Wolf ET Source Code.  If not, see <http://www.gnu.org/licenses/>.

In addition, the Wolf: ET Source Code is also subject to certain additional terms. You should have received a copy of these additional terms immediately following the terms and conditions of the GNU General Public License which accompanied the Wolf ET Source Code.  If not, please request a copy in writing from id Software at the address below.

If you have questions concerning this license or the applicable additional terms, you may contact in writing id Software LLC, c/o ZeniMax Media Inc., Suite 120, Rockville, Maryland 20850 USA.

===========================================================================
*/

#include "g_local.h"
#include "g_api.h"

level_locals_t level;

typedef struct {
	vmCvar_t    *vmCvar;
	char        *cvarName;
	char        *defaultString;
	int cvarFlags;
	int modificationCount;          // for tracking changes
	qboolean trackChange;           // track this variable, and announce if changed
	qboolean fConfigReset;          // OSP: set this var to the default on a config reset
	qboolean teamShader;            // track and if changed, update shader state
} cvarTable_t;

gentity_t g_entities[MAX_GENTITIES];
gclient_t g_clients[MAX_CLIENTS];

g_campaignInfo_t g_campaigns[MAX_CAMPAIGNS];

mapEntityData_Team_t mapEntityData[2];

vmCvar_t g_password;
vmCvar_t sv_privatepassword;
vmCvar_t g_maxclients;
vmCvar_t g_dedicated;
vmCvar_t g_inactivity;
vmCvar_t g_debugMove;
vmCvar_t g_debugDamage;
vmCvar_t g_debugAlloc;
vmCvar_t g_debugBullets;    //----(SA)	added
vmCvar_t g_motd;
vmCvar_t g_gamestate;
vmCvar_t g_swapteams;
// -NERVE - SMF

vmCvar_t g_restarted;
vmCvar_t g_log;
vmCvar_t g_logSync;
vmCvar_t voteFlags;
vmCvar_t g_filtercams;
vmCvar_t g_voiceChatsAllowed;       // DHM - Nerve
vmCvar_t g_needpass;
vmCvar_t g_banIPs;
vmCvar_t g_filterBan;
vmCvar_t g_smoothClients;
vmCvar_t pmove_fixed;
vmCvar_t pmove_msec;

// Rafael
vmCvar_t g_scriptName;          // name of script file to run (instead of default for that map)

vmCvar_t g_developer;
vmCvar_t g_footstepAudibleRange;

// charge times for character class special weapons
vmCvar_t g_medicChargeTime;
vmCvar_t g_engineerChargeTime;
vmCvar_t g_LTChargeTime;
vmCvar_t g_soldierChargeTime;
// screen shakey magnitude multiplier

// Gordon
vmCvar_t g_antilag;

// OSP
vmCvar_t g_spectatorInactivity;
vmCvar_t team_maxPanzers;
vmCvar_t team_maxplayers;
vmCvar_t team_nocontrols;
vmCvar_t server_motd0;
vmCvar_t server_motd1;
vmCvar_t server_motd2;
vmCvar_t server_motd3;
vmCvar_t server_motd4;
vmCvar_t server_motd5;
vmCvar_t vote_allow_kick;
vmCvar_t vote_allow_map;
vmCvar_t vote_allow_matchreset;
vmCvar_t vote_allow_mutespecs;
vmCvar_t vote_allow_nextmap;
vmCvar_t vote_allow_referee;
vmCvar_t vote_allow_swapteams;
vmCvar_t vote_allow_antilag;
vmCvar_t vote_allow_muting;
vmCvar_t vote_limit;
vmCvar_t vote_percent;
vmCvar_t g_covertopsChargeTime;
vmCvar_t refereePassword;
vmCvar_t g_debugConstruct;

// Variable for setting the current level of debug printing/logging
// enabled in bot scripts and regular scripts.
// Added by Mad Doctor I, 8/23/2002
vmCvar_t g_scriptDebugLevel;
vmCvar_t mod_url;
vmCvar_t url;

vmCvar_t g_letterbox;

vmCvar_t g_debugSkills;
vmCvar_t g_heavyWeaponRestriction;
vmCvar_t g_nextmap;

// Nico, beginning of ETrun server cvars

// Max connections per IP
vmCvar_t g_maxConnsPerIP;

// Game physics
vmCvar_t physics;

// Enable certain map entities
vmCvar_t g_enableMapEntities;

// Force timer reset, i.e. bypass "wait 9999" on start triggers
vmCvar_t g_forceTimerReset;

// Is level a timerun?
vmCvar_t isTimerun;

// Flood protection
vmCvar_t g_floodProtect;
vmCvar_t g_floodThreshold;
vmCvar_t g_floodWait;

// Name changes limit
vmCvar_t g_maxNameChanges;

// API module
vmCvar_t g_useAPI;
vmCvar_t g_APImodulePath;

// Nico, end of ETrun cvars


cvarTable_t gameCvarTable[] = {
	// noset vars
	{ NULL, "gamename", GAME_VERSION, CVAR_SERVERINFO | CVAR_ROM, 0, qfalse  },
	{ NULL, "gamedate", __DATE__, CVAR_ROM, 0, qfalse  },
	{ &g_restarted, "g_restarted", "0", CVAR_ROM, 0, qfalse  },
	{ NULL, "sv_mapname", "", CVAR_SERVERINFO | CVAR_ROM, 0, qfalse  },

	// latched vars

	{ &g_medicChargeTime, "g_medicChargeTime", "45000", CVAR_SERVERINFO | CVAR_LATCH, 0, qfalse, qtrue },
	{ &g_engineerChargeTime, "g_engineerChargeTime", "30000", CVAR_SERVERINFO | CVAR_LATCH, 0, qfalse, qtrue },
	{ &g_LTChargeTime, "g_LTChargeTime", "40000", CVAR_SERVERINFO | CVAR_LATCH, 0, qfalse, qtrue },
	{ &g_soldierChargeTime, "g_soldierChargeTime", "20000", CVAR_SERVERINFO | CVAR_LATCH, 0, qfalse, qtrue },

	{ &g_covertopsChargeTime, "g_covertopsChargeTime", "30000", CVAR_SERVERINFO | CVAR_LATCH, 0, qfalse, qtrue },
	{ &g_maxclients, "sv_maxclients", "20", CVAR_SERVERINFO | CVAR_LATCH | CVAR_ARCHIVE, 0, qfalse  },            // NERVE - SMF - made 20 from 8

	{ &g_gamestate, "gamestate", "-1", CVAR_WOLFINFO | CVAR_ROM, 0, qfalse  },

	{ &g_swapteams, "g_swapteams", "0", CVAR_ROM, 0, qfalse, qtrue },
	// -NERVE - SMF

	{ &g_log, "g_log", "", CVAR_ARCHIVE, 0, qfalse },
	{ &g_logSync, "g_logSync", "0", CVAR_ARCHIVE, 0, qfalse },

	{ &g_password, "g_password", "none", CVAR_USERINFO, 0, qfalse },
	{ &sv_privatepassword, "sv_privatepassword", "", CVAR_TEMP, 0, qfalse },
	{ &g_banIPs, "g_banIPs", "", CVAR_ARCHIVE, 0, qfalse },
	// show_bug.cgi?id=500
	{ &g_filterBan, "g_filterBan", "1", CVAR_ARCHIVE, 0, qfalse },

	{ &g_dedicated, "dedicated", "0", 0, 0, qfalse },

	{ &g_needpass, "g_needpass", "0", CVAR_SERVERINFO | CVAR_ROM, 0, qtrue },

	{ &g_inactivity, "g_inactivity", "120", 0, 0, qtrue },// Nico, set to 120 secs (default was 0)
	{ &g_debugMove, "g_debugMove", "0", 0, 0, qfalse },
	{ &g_debugDamage, "g_debugDamage", "0", CVAR_CHEAT, 0, qfalse },
	{ &g_debugAlloc, "g_debugAlloc", "0", 0, 0, qfalse },
	{ &g_debugBullets, "g_debugBullets", "0", CVAR_CHEAT, 0, qfalse}, //----(SA)	added
	{ &g_motd, "g_motd", "", CVAR_ARCHIVE, 0, qfalse },
	{ &voteFlags, "voteFlags", "0", CVAR_TEMP | CVAR_ROM | CVAR_SERVERINFO, 0, qfalse },
	{ &g_filtercams, "g_filtercams", "0", CVAR_ARCHIVE, 0, qfalse },
	{ &g_voiceChatsAllowed, "g_voiceChatsAllowed", "4", CVAR_ARCHIVE, 0, qfalse },                // DHM - Nerve
	{ &g_developer, "developer", "0", CVAR_TEMP, 0, qfalse },
	{ &g_smoothClients, "g_smoothClients", "1", 0, 0, qfalse },
	{ &pmove_fixed, "pmove_fixed", "0", 0, 0, qfalse },// Nico, removed CVAR_SYSTEMINFO
	{ &pmove_msec, "pmove_msec", "8", CVAR_SYSTEMINFO, 0, qfalse },

	{ &g_footstepAudibleRange, "g_footstepAudibleRange", "256", CVAR_CHEAT, 0, qfalse },

	{ &g_scriptName, "g_scriptName", "", CVAR_CHEAT, 0, qfalse },

	{ &g_antilag, "g_antilag", "1", CVAR_SERVERINFO | CVAR_ARCHIVE, 0, qfalse },

	//bani - #184
	{ NULL, "P", "", CVAR_SERVERINFO_NOUPDATE, 0, qfalse, qfalse },

	{ &refereePassword, "refereePassword", "none", 0, 0, qfalse},
	{ &g_spectatorInactivity, "g_spectatorInactivity", "0", 0, 0, qfalse, qfalse },

	{ &server_motd0,    "server_motd0", " ^NEnemy Territory ^7MOTD ", 0, 0, qfalse, qfalse },
	{ &server_motd1,    "server_motd1", "", 0, 0, qfalse, qfalse },
	{ &server_motd2,    "server_motd2", "", 0, 0, qfalse, qfalse },
	{ &server_motd3,    "server_motd3", "", 0, 0, qfalse, qfalse },
	{ &server_motd4,    "server_motd4", "", 0, 0, qfalse, qfalse },
	{ &server_motd5,    "server_motd5", "", 0, 0, qfalse, qfalse },
	{ &team_maxPanzers, "team_maxPanzers", "-1", 0, 0, qfalse, qfalse },
	{ &team_maxplayers, "team_maxplayers", "0", 0, 0, qfalse, qfalse },
	{ &team_nocontrols, "team_nocontrols", "1", 0, 0, qfalse, qfalse },

	{ &vote_allow_kick,         "vote_allow_kick", "1", 0, 0, qfalse, qfalse },
	{ &vote_allow_map,          "vote_allow_map", "1", 0, 0, qfalse, qfalse },
	{ &vote_allow_matchreset,   "vote_allow_matchreset", "1", 0, 0, qfalse, qfalse },
	{ &vote_allow_mutespecs,    "vote_allow_mutespecs", "1", 0, 0, qfalse, qfalse },
	{ &vote_allow_nextmap,      "vote_allow_nextmap", "1", 0, 0, qfalse, qfalse },

	{ &vote_allow_referee,      "vote_allow_referee", "0", 0, 0, qfalse, qfalse },

	{ &vote_allow_swapteams,    "vote_allow_swapteams", "1", 0, 0, qfalse, qfalse },

	{ &vote_allow_antilag,      "vote_allow_antilag", "1", 0, 0, qfalse, qfalse },

	{ &vote_allow_muting,       "vote_allow_muting", "1", 0, 0, qfalse, qfalse },
	{ &vote_limit,      "vote_limit", "5", 0, 0, qfalse, qfalse },
	{ &vote_percent,    "vote_percent", "50", 0, 0, qfalse, qfalse },

	// state vars
	{ &g_debugConstruct, "g_debugConstruct", "0", CVAR_CHEAT, 0, qfalse },

	{ &g_scriptDebug, "g_scriptDebug", "0", CVAR_CHEAT, 0, qfalse },

	// What level of detail do we want script printing to go to.
	{ &g_scriptDebugLevel, "g_scriptDebugLevel", "0", CVAR_CHEAT, 0, qfalse },

	// points to the URL for mod information, should not be modified by server admin
	{ &mod_url, "mod_url", "", CVAR_SERVERINFO | CVAR_ROM, 0, qfalse },
	// configured by the server admin, points to the web pages for the server
	{ &url, "URL", "", CVAR_SERVERINFO | CVAR_ARCHIVE, 0, qfalse },

	{ &g_letterbox, "cg_letterbox", "0", CVAR_TEMP    },

	{ &g_debugSkills,   "g_debugSkills", "0", 0       },

	{ &g_heavyWeaponRestriction, "g_heavyWeaponRestriction", "100", CVAR_ARCHIVE | CVAR_SERVERINFO },

	{ &g_nextmap, "nextmap", "", CVAR_TEMP },

	// Nico, beginning of ETrun server cvars

	// Max connections per IP
	{ &g_maxConnsPerIP, "g_maxConnsPerIP", "3", CVAR_SERVERINFO | CVAR_ARCHIVE},

	// Game physics (set serverside but sent to client for prediction)
	{ &physics, "physics", "15", CVAR_SERVERINFO | CVAR_ARCHIVE | CVAR_LATCH | CVAR_SYSTEMINFO},

	// Enable certain map entities
	// 3 enabled both kill entities and hurt entities
	{ &g_enableMapEntities, "g_enableMapEntities", "31", CVAR_SERVERINFO | CVAR_ARCHIVE | CVAR_LATCH},

	// Force timer reset, i.e. bypass "wait 9999" on start triggers
	{ &g_forceTimerReset, "g_forceTimerReset", "1", CVAR_SERVERINFO | CVAR_ARCHIVE | CVAR_LATCH},

	// Is level a timerun?
	{ &isTimerun, "isTimerun", "0", CVAR_ROM | CVAR_SYSTEMINFO },

	// Flood protection
	{ &g_floodProtect, "g_floodProtect", "1", 0 },
	{ &g_floodThreshold, "g_floodThreshold", "8", 0 },
	{ &g_floodWait, "g_floodWait", "768", 0 },

	// Name changes limit
	{ &g_maxNameChanges, "g_maxNameChanges", "3", 0},

	// API module
	{ &g_useAPI, "g_useAPI", "0", CVAR_ARCHIVE | CVAR_LATCH},
	{ &g_APImodulePath, "g_APImodulePath", "", CVAR_ARCHIVE | CVAR_LATCH}

	// Nico, end of ETrun cvars
};

// bk001129 - made static to avoid aliasing
static int gameCvarTableSize = sizeof( gameCvarTable ) / sizeof( gameCvarTable[0] );


void G_InitGame( int levelTime, int randomSeed, int restart );
void G_RunFrame( int levelTime );
void G_ShutdownGame( int restart );

/*
================
vmMain

This is the only way control passes into the module.
This must be the very first function compiled into the .q3vm file
================
*/
#if __GNUC__ >= 4
#pragma GCC visibility push(default)
#endif
int vmMain( int command, int arg0, int arg1, int arg2, int arg3, int arg4, int arg5, int arg6 ) {
#if __GNUC__ >= 4
#pragma GCC visibility pop
#endif
	switch ( command ) {
	case GAME_INIT:
		G_InitGame( arg0, arg1, arg2 );
		return 0;
	case GAME_SHUTDOWN:
		G_ShutdownGame( arg0 );
		return 0;
	case GAME_CLIENT_CONNECT:
		return (int)ClientConnect( arg0, arg1, arg2 );
	case GAME_CLIENT_THINK:
		ClientThink( arg0 );
		return 0;
	case GAME_CLIENT_USERINFO_CHANGED:
		ClientUserinfoChanged( arg0 );
		return 0;
	case GAME_CLIENT_DISCONNECT:
		ClientDisconnect( arg0 );
		return 0;
	case GAME_CLIENT_BEGIN:
		ClientBegin( arg0 );
		return 0;
	case GAME_CLIENT_COMMAND:
		ClientCommand( arg0 );
		return 0;
	case GAME_RUN_FRAME:
		G_RunFrame( arg0 );
		return 0;
	case GAME_CONSOLE_COMMAND:
		return ConsoleCommand();
	case GAME_SNAPSHOT_CALLBACK:
		return qtrue;

	case GAME_MESSAGERECEIVED:
		return -1;
	}

	return -1;
}

void QDECL G_Printf( const char *fmt, ... ) {
	va_list argptr;
	char text[1024];

	va_start( argptr, fmt );
	Q_vsnprintf( text, sizeof( text ), fmt, argptr );
	va_end( argptr );

	trap_Printf( text );
}
//bani
void QDECL G_Printf( const char *fmt, ... ) _attribute( ( format( printf,1,2 ) ) );

void QDECL G_DPrintf( const char *fmt, ... ) {
	va_list argptr;
	char text[1024];

	if ( !g_developer.integer ) {
		return;
	}

	va_start( argptr, fmt );
	Q_vsnprintf( text, sizeof( text ), fmt, argptr );
	va_end( argptr );

	trap_Printf( text );
}
//bani
void QDECL G_DPrintf( const char *fmt, ... ) _attribute( ( format( printf,1,2 ) ) );

void QDECL G_Error( const char *fmt, ... ) {
	va_list argptr;
	char text[1024];

	va_start( argptr, fmt );
	Q_vsnprintf( text, sizeof( text ), fmt, argptr );
	va_end( argptr );

	trap_Error( text );
}
//bani
void QDECL G_Error( const char *fmt, ... ) _attribute( ( format( printf,1,2 ) ) );


#define CH_KNIFE_DIST       48  // from g_weapon.c
#define CH_LADDER_DIST      100
#define CH_WATER_DIST       100
#define CH_BREAKABLE_DIST   64
#define CH_DOOR_DIST        96
#define CH_ACTIVATE_DIST    96
#define CH_EXIT_DIST        256
#define CH_FRIENDLY_DIST    1024

#define CH_MAX_DIST         1024    // use the largest value from above
#define CH_MAX_DIST_ZOOM    8192    // max dist for zooming hints

/*
==============
G_CursorHintIgnoreEnt: returns whether the ent should be ignored
for cursor hint purpose (because the ent may have the designed content type
but nevertheless should not display any cursor hint)
==============
*/
static qboolean G_CursorHintIgnoreEnt( gentity_t *traceEnt, gentity_t *clientEnt ) {
	return ( traceEnt->s.eType == ET_OID_TRIGGER || traceEnt->s.eType == ET_TRIGGER_MULTIPLE ) ? qtrue : qfalse;
}

qboolean G_EmplacedGunIsMountable( gentity_t* ent, gentity_t* other ) {
	if ( Q_stricmp( ent->classname, "misc_mg42" ) && Q_stricmp( ent->classname, "misc_aagun" ) ) {
		return qfalse;
	}

	if ( !other->client ) {
		return qfalse;
	}

	if ( BG_IsScopedWeapon( other->client->ps.weapon ) ) {
		return qfalse;
	}

	if ( other->client->ps.pm_flags & PMF_DUCKED ) {
		return qfalse;
	}

	if ( other->client->ps.persistant[PERS_HWEAPON_USE] ) {
		return qfalse;
	}

	if ( ent->r.currentOrigin[2] - other->r.currentOrigin[2] >= 40 ) {
		return qfalse;
	}

	if ( ent->r.currentOrigin[2] - other->r.currentOrigin[2] < 0 ) {
		return qfalse;
	}

	if ( ent->s.frame != 0 ) {
		return qfalse;
	}

	if ( ent->active ) {
		return qfalse;
	}

	if ( other->client->ps.grenadeTimeLeft ) {
		return qfalse;
	}

	if ( infront( ent, other ) ) {
		return qfalse;
	}

	return qtrue;
}

qboolean G_EmplacedGunIsRepairable( gentity_t* ent, gentity_t* other ) {
	if ( Q_stricmp( ent->classname, "misc_mg42" ) && Q_stricmp( ent->classname, "misc_aagun" ) ) {
		return qfalse;
	}

	if ( !other->client ) {
		return qfalse;
	}

	if ( BG_IsScopedWeapon( other->client->ps.weapon ) ) {
		return qfalse;
	}

	if ( other->client->ps.persistant[PERS_HWEAPON_USE] ) {
		return qfalse;
	}

	if ( ent->s.frame == 0 ) {
		return qfalse;
	}

	return qtrue;
}

void G_CheckForCursorHints( gentity_t *ent ) {
	vec3_t forward, right, up, offset, end;
	trace_t     *tr;
	float dist;
	float chMaxDist = CH_MAX_DIST;
	gentity_t   *checkEnt, *traceEnt = 0;
	playerState_t *ps;
	int hintType, hintDist, hintVal;
	qboolean zooming;
	int trace_contents;                 // DHM - Nerve
	int numOfIgnoredEnts = 0;

	if ( !ent->client ) {
		return;
	}

	ps = &ent->client->ps;

	zooming = (qboolean)( ps->eFlags & EF_ZOOMING );

	AngleVectors( ps->viewangles, forward, right, up );

	VectorCopy( ps->origin, offset );
	offset[2] += ps->viewheight;

	// lean
	if ( ps->leanf ) {
		VectorMA( offset, ps->leanf, right, offset );
	}

	if ( zooming ) {
		VectorMA( offset, CH_MAX_DIST_ZOOM, forward, end );
	} else {
		VectorMA( offset, chMaxDist, forward, end );
	}

	tr = &ps->serverCursorHintTrace;

	trace_contents = ( CONTENTS_TRIGGER | CONTENTS_SOLID | CONTENTS_MISSILECLIP | CONTENTS_BODY | CONTENTS_CORPSE );
	trap_Trace( tr, offset, NULL, NULL, end, ps->clientNum, trace_contents );

	// reset all
	hintType    = ps->serverCursorHint      = HINT_NONE;
	hintVal     = ps->serverCursorHintVal   = 0;

	dist = VectorDistanceSquared( offset, tr->endpos );

	if ( zooming ) {
		hintDist    = CH_MAX_DIST_ZOOM;
	} else {
		hintDist    = chMaxDist;
	}

	// Arnout: building something - add this here because we don't have anything solid to trace to - quite ugly-ish
	if ( ent->client->touchingTOI && ps->stats[ STAT_PLAYER_CLASS ] == PC_ENGINEER ) {
		gentity_t* constructible;
		if ( ( constructible = G_IsConstructible( ent->client->sess.sessionTeam, ent->client->touchingTOI ) ) ) {
			ps->serverCursorHint = HINT_CONSTRUCTIBLE;
			ps->serverCursorHintVal = (int)constructible->s.angles2[0];
			return;
		}
	}

	if ( tr->fraction == 1 ) {
		return;
	}

	traceEnt = &g_entities[tr->entityNum];
	while ( G_CursorHintIgnoreEnt( traceEnt, ent ) && numOfIgnoredEnts < 10 ) {
		// xkan, 1/9/2003 - we may hit multiple invalid ents at the same point
		// count them to prevent too many loops
		numOfIgnoredEnts++;

		// xkan, 1/8/2003 - advance offset (start point) past the entity to ignore
		VectorMA( tr->endpos, 0.1, forward, offset );

		trap_Trace( tr, offset, NULL, NULL, end, traceEnt->s.number, trace_contents );

		// xkan, 1/8/2003 - (hintDist - dist) is the actual distance in the above
		// trap_Trace call. update dist accordingly.
		dist += VectorDistanceSquared( offset, tr->endpos );
		if ( tr->fraction == 1 ) {
			return;
		}
		traceEnt = &g_entities[tr->entityNum];
	}

	if ( tr->entityNum == ENTITYNUM_WORLD ) {
		if ( ( tr->contents & CONTENTS_WATER ) && !( ps->powerups[PW_BREATHER] ) ) {
			hintDist = CH_WATER_DIST;
			hintType = HINT_WATER;
		} else if ( ( tr->surfaceFlags & SURF_LADDER ) && !( ps->pm_flags & PMF_LADDER ) ) { // ladder
			hintDist = CH_LADDER_DIST;
			hintType = HINT_LADDER;
		}
	} else if ( tr->entityNum < MAX_CLIENTS ) {
		// Show medics a syringe if they can revive someone

		if ( traceEnt->client && traceEnt->client->sess.sessionTeam == ent->client->sess.sessionTeam ) {
			if ( ps->stats[ STAT_PLAYER_CLASS ] == PC_MEDIC && traceEnt->client->ps.pm_type == PM_DEAD && !( traceEnt->client->ps.pm_flags & PMF_LIMBO ) ) {
				hintDist    = 48;     // JPW NERVE matches weapon_syringe in g_weapon.c
				hintType    = HINT_REVIVE;
			}
		} else if ( traceEnt->client && traceEnt->client->isCivilian ) {
			// xkan, 1/6/2003 - check for civilian, show neutral cursor (no matter which team)
			hintType = HINT_PLYR_NEUTRAL;
			hintDist = CH_FRIENDLY_DIST;    // far, since this will be used to determine whether to shoot bullet weaps or not
		}
	} else {
		checkEnt = traceEnt;

		// Arnout: invisible entities don't show hints
		if ( traceEnt->entstate == STATE_INVISIBLE || traceEnt->entstate == STATE_UNDERCONSTRUCTION ) {
			return;
		}

		// check invisible_users first since you don't want to draw a hint based
		// on that ent, but rather on what they are targeting.
		// so find the target and set checkEnt to that to show the proper hint.
		if ( traceEnt->s.eType == ET_GENERAL ) {

			// ignore trigger_aidoor.  can't just not trace for triggers, since I need invisible_users...
			// damn, I would like to ignore some of these triggers though.

			if ( !Q_stricmp( traceEnt->classname, "trigger_aidoor" ) ) {
				return;
			}

			if ( !Q_stricmp( traceEnt->classname, "func_invisible_user" ) ) {
				// indirectHit = qtrue;

				// DHM - Nerve :: Put this back in only in multiplayer
				if ( traceEnt->s.dmgFlags ) {  // hint icon specified in entity
					hintType = traceEnt->s.dmgFlags;
					hintDist = CH_ACTIVATE_DIST;
					checkEnt = 0;
				} else { // use target for hint icon
					checkEnt = G_FindByTargetname( NULL, traceEnt->target );
					if ( !checkEnt ) {     // no target found
						hintType = HINT_BAD_USER;
						hintDist = CH_MAX_DIST_ZOOM;    // show this one from super far for debugging
					}
				}
			}
		}


		if ( checkEnt ) {

			// TDF This entire function could be the poster boy for converting to OO programming!!!
			// I'm making this into a switch in a vain attempt to make this readable so I can find which
			// brackets don't match!!!

			switch ( checkEnt->s.eType ) {
			case ET_CORPSE:
				if ( !ent->client->ps.powerups[PW_BLUEFLAG] && !ent->client->ps.powerups[PW_REDFLAG] ) {
					if ( BODY_TEAM( traceEnt ) < 4 && BODY_TEAM( traceEnt ) != ent->client->sess.sessionTeam && traceEnt->nextthink == traceEnt->timestamp + BODY_TIME( BODY_TEAM( traceEnt ) ) ) {
						if ( ent->client->ps.stats[STAT_PLAYER_CLASS] == PC_COVERTOPS ) {
							hintDist    = 48;
							hintType    = HINT_UNIFORM;
							hintVal     = BODY_VALUE( traceEnt );
							if ( hintVal > 255 ) {
								hintVal = 255;
							}
						}
					}
				}
				break;
			case ET_GENERAL:
			case ET_MG42_BARREL:
			case ET_AAGUN:
				hintType = HINT_FORCENONE;

				if ( G_EmplacedGunIsMountable( traceEnt, ent ) ) {
					hintDist = CH_ACTIVATE_DIST;
					hintType = HINT_MG42;
					hintVal = 0;
				} else {
					if ( ps->stats[ STAT_PLAYER_CLASS ] == PC_ENGINEER && G_EmplacedGunIsRepairable( traceEnt, ent ) ) {
						hintType = HINT_BUILD;
						hintDist = CH_BREAKABLE_DIST;
						hintVal = traceEnt->health;
						if ( hintVal > 255 ) {
							hintVal = 255;
						}
					} else {
						hintDist = 0;
						hintType = ps->serverCursorHint = HINT_FORCENONE;
						hintVal = ps->serverCursorHintVal = 0;
					}
				}
				break;
			case ET_EXPLOSIVE:
			{
				if ( checkEnt->spawnflags & EXPLOSIVE_TANK ) {
					hintDist = CH_BREAKABLE_DIST * 2;
					hintType = HINT_TANK;
					hintVal  = ps->serverCursorHintVal  = 0;        // no health for tank destructibles
				} else {
					switch ( checkEnt->constructibleStats.weaponclass ) {
					case 0:
						hintDist = CH_BREAKABLE_DIST;
						hintType = HINT_BREAKABLE;
						hintVal  = checkEnt->health;            // also send health to client for visualization
						break;
					case 1:
						hintDist = CH_BREAKABLE_DIST * 2;
						hintType = HINT_SATCHELCHARGE;
						hintVal  = ps->serverCursorHintVal = 0;     // no health for satchel charges
						break;
					case 2:
						hintDist = 0;
						hintType = ps->serverCursorHint = HINT_FORCENONE;
						hintVal = ps->serverCursorHintVal = 0;

						if ( checkEnt->parent && checkEnt->parent->s.eType == ET_OID_TRIGGER ) {
							if ( ( ( ent->client->sess.sessionTeam == TEAM_AXIS ) && ( checkEnt->parent->spawnflags & ALLIED_OBJECTIVE ) ) ||
								 ( ( ent->client->sess.sessionTeam == TEAM_ALLIES ) && ( checkEnt->parent->spawnflags & AXIS_OBJECTIVE ) ) ) {
								hintDist = CH_BREAKABLE_DIST * 2;
								hintType = HINT_BREAKABLE_DYNAMITE;
								hintVal  = ps->serverCursorHintVal  = 0;        // no health for dynamite
							}
						}
						break;
					default:
						if ( checkEnt->health > 0 ) {
							hintDist = CH_BREAKABLE_DIST;
							hintType = HINT_BREAKABLE;
							hintVal  = checkEnt->health;            // also send health to client for visualization
						} else {
							hintDist = 0;
							hintType = ps->serverCursorHint     = HINT_FORCENONE;
							hintVal  = ps->serverCursorHintVal  = 0;
						}
						break;
					}
				}

				break;
			}
			case ET_CONSTRUCTIBLE:
				if ( G_ConstructionIsPartlyBuilt( checkEnt ) && !( checkEnt->spawnflags & CONSTRUCTIBLE_INVULNERABLE ) ) {
					// only show hint for players who can blow it up
					if ( checkEnt->s.teamNum != ent->client->sess.sessionTeam ) {
						switch ( checkEnt->constructibleStats.weaponclass ) {
						case 0:
							hintDist = CH_BREAKABLE_DIST;
							hintType = HINT_BREAKABLE;
							hintVal  = checkEnt->health;            // also send health to client for visualization
							break;
						case 1:
							hintDist = CH_BREAKABLE_DIST * 2;
							hintType = HINT_SATCHELCHARGE;
							hintVal  = ps->serverCursorHintVal  = 0;        // no health for satchel charges
							break;
						case 2:
							hintDist = CH_BREAKABLE_DIST * 2;
							hintType = HINT_BREAKABLE_DYNAMITE;
							hintVal  = ps->serverCursorHintVal  = 0;        // no health for dynamite
							break;
						default:
							hintDist = 0;
							hintType = ps->serverCursorHint     = HINT_FORCENONE;
							hintVal  = ps->serverCursorHintVal  = 0;
							break;
						}
					} else {
						hintDist = 0;
						hintType = ps->serverCursorHint     = HINT_FORCENONE;
						hintVal  = ps->serverCursorHintVal  = 0;
						return;
					}
				}

				break;
			case ET_ALARMBOX:
				if ( checkEnt->health > 0 ) {
					hintType = HINT_ACTIVATE;
				}
				break;

			case ET_ITEM:
			{
				gitem_t* it = &bg_itemlist[checkEnt->item - bg_itemlist];

				hintDist = CH_ACTIVATE_DIST;

				switch ( it->giType ) {
				case IT_HEALTH:
					hintType = HINT_HEALTH;
					break;
				case IT_TREASURE:
					hintType = HINT_TREASURE;
					break;
				case IT_WEAPON: {
					qboolean canPickup = COM_BitCheck( ent->client->ps.weapons, it->giTag );

					if ( !canPickup ) {
						if ( it->giTag == WP_AMMO ) {
							canPickup = qtrue;
						}
					}

					if ( !canPickup ) {
						canPickup = G_CanPickupWeapon( it->giTag, ent );
					}

					if ( canPickup ) {
						hintType = HINT_WEAPON;
					}
					break;
				}
				case IT_AMMO:
					hintType = HINT_AMMO;
					break;
				case IT_ARMOR:
					hintType = HINT_ARMOR;
					break;
				case IT_HOLDABLE:
					hintType = HINT_HOLDABLE;
					break;
				case IT_KEY:
					hintType = HINT_INVENTORY;
					break;
				case IT_TEAM:
					if ( !Q_stricmp( traceEnt->classname, "team_CTF_redflag" ) && ent->client->sess.sessionTeam == TEAM_ALLIES ) {
						hintType = HINT_POWERUP;
					} else if ( !Q_stricmp( traceEnt->classname, "team_CTF_blueflag" ) && ent->client->sess.sessionTeam == TEAM_AXIS ) {
						hintType = HINT_POWERUP;
					}
					break;
				case IT_BAD:
				default:
					break;
				}

				break;
			}
			case ET_MOVER:
				if ( !Q_stricmp( checkEnt->classname, "script_mover" ) ) {
					if ( G_TankIsMountable( checkEnt, ent ) ) {
						hintDist = CH_ACTIVATE_DIST;
						hintType = HINT_ACTIVATE;
					}
				} else if ( !Q_stricmp( checkEnt->classname, "func_door_rotating" ) ) {
					if ( checkEnt->moverState == MOVER_POS1ROTATE ) {    // stationary/closed
						hintDist = CH_DOOR_DIST;
						hintType = HINT_DOOR_ROTATING;
						if ( checkEnt->key == -1 || !G_AllowTeamsAllowed( checkEnt, ent ) ) {    // locked
							hintType = HINT_DOOR_ROTATING_LOCKED;
						}
					}
				} else if ( !Q_stricmp( checkEnt->classname, "func_door" ) ) {
					if ( checkEnt->moverState == MOVER_POS1 ) {  // stationary/closed
						hintDist = CH_DOOR_DIST;
						hintType = HINT_DOOR;

						if ( checkEnt->key == -1 || !G_AllowTeamsAllowed( checkEnt, ent ) ) {    // locked
							hintType = HINT_DOOR_LOCKED;
						}
					}
				} else if ( !Q_stricmp( checkEnt->classname, "func_button" ) ) {
					hintDist = CH_ACTIVATE_DIST;
					hintType = HINT_BUTTON;
				} else if ( !Q_stricmp( checkEnt->classname, "props_flamebarrel" ) ) {
					hintDist = CH_BREAKABLE_DIST * 2;
					hintType = HINT_BREAKABLE;
				} else if ( !Q_stricmp( checkEnt->classname, "props_statue" ) ) {
					hintDist = CH_BREAKABLE_DIST * 2;
					hintType = HINT_BREAKABLE;
				}

				break;
			case ET_MISSILE:
			case ET_BOMB:
				if ( ps->stats[ STAT_PLAYER_CLASS ] == PC_ENGINEER ) {
					hintDist    = CH_BREAKABLE_DIST;
					hintType    = HINT_DISARM;
					hintVal     = checkEnt->health;         // also send health to client for visualization
					if ( hintVal > 255 ) {
						hintVal = 255;
					}
				}


				// hint icon specified in entity (and proper contact was made, so hintType was set)
				// first try the checkent...
				if ( checkEnt->s.dmgFlags && hintType ) {
					hintType = checkEnt->s.dmgFlags;
				}

				// then the traceent
				if ( traceEnt->s.dmgFlags && hintType ) {
					hintType = traceEnt->s.dmgFlags;
				}

				break;
			default:
				break;
			}

			if ( zooming ) {
				hintDist = CH_MAX_DIST_ZOOM;

				// zooming can eat a lot of potential hints
				switch ( hintType ) {

					// allow while zooming
				case HINT_PLAYER:
				case HINT_TREASURE:
				case HINT_LADDER:
				case HINT_EXIT:
				case HINT_NOEXIT:
				case HINT_PLYR_FRIEND:
				case HINT_PLYR_NEUTRAL:
				case HINT_PLYR_ENEMY:
				case HINT_PLYR_UNKNOWN:
					break;

				default:
					return;
				}
			}
		}
	}

	if ( dist <= Square( hintDist ) ) {
		ps->serverCursorHint = hintType;
		ps->serverCursorHintVal = hintVal;
	}

}

void G_SetTargetName( gentity_t* ent, char* targetname ) {
	if ( targetname && *targetname ) {
		ent->targetname = targetname;
		ent->targetnamehash = BG_StringHashValue( targetname );
	} else {
		ent->targetnamehash = -1;
	}
}

/*
================
G_FindTeams

Chain together all entities with a matching team field.
Entity teams are used for item groups and multi-entity mover groups.

All but the first will have the FL_TEAMSLAVE flag set and teammaster field set
All but the last will have the teamchain field set to the next one
================
*/
void G_FindTeams( void ) {
	gentity_t   *e, *e2;
	int i, j;
	int c, c2;

	c = 0;
	c2 = 0;
	for ( i = 1, e = g_entities + i ; i < level.num_entities ; i++,e++ ) {
		if ( !e->inuse ) {
			continue;
		}

		if ( !e->team ) {
			continue;
		}

		if ( e->flags & FL_TEAMSLAVE ) {
			continue;
		}

		if ( !Q_stricmp( e->classname, "func_tramcar" ) ) {
			if ( e->spawnflags & 8 ) { // leader
				e->teammaster = e;
			} else
			{
				continue;
			}
		}

		c++;
		c2++;
		for ( j = i + 1, e2 = e + 1 ; j < level.num_entities ; j++,e2++ )
		{
			if ( !e2->inuse ) {
				continue;
			}
			if ( !e2->team ) {
				continue;
			}
			if ( e2->flags & FL_TEAMSLAVE ) {
				continue;
			}
			if ( !strcmp( e->team, e2->team ) ) {
				c2++;
				e2->teamchain = e->teamchain;
				e->teamchain = e2;
				e2->teammaster = e;
//				e2->key = e->key;	// (SA) I can't set the key here since the master door hasn't finished spawning yet and therefore has a key of -1
				e2->flags |= FL_TEAMSLAVE;

				if ( !Q_stricmp( e2->classname, "func_tramcar" ) ) {
					trap_UnlinkEntity( e2 );
				}

				// make sure that targets only point at the master
				if ( e2->targetname ) {
					G_SetTargetName( e, e2->targetname );

					// Rafael
					// note to self: added this because of problems
					// pertaining to keys and double doors
					if ( Q_stricmp( e2->classname, "func_door_rotating" ) ) {
						e2->targetname = NULL;
					}
				}
			}
		}
	}

	G_Printf( "%i teams with %i entities\n", c, c2 );
}


/*
==============
G_RemapTeamShaders
==============
*/
void G_RemapTeamShaders() {
}


/*
=================
G_RegisterCvars
=================
*/
void G_RegisterCvars( void ) {
	int i;
	cvarTable_t *cv;
	qboolean remapped = qfalse;

	level.server_settings = 0;

	for ( i = 0, cv = gameCvarTable; i < gameCvarTableSize; i++, cv++ ) {
		trap_Cvar_Register( cv->vmCvar, cv->cvarName, cv->defaultString, cv->cvarFlags );
		if ( cv->vmCvar ) {
			cv->modificationCount = cv->vmCvar->modificationCount;
			// OSP - Update vote info for clients, if necessary
			G_checkServerToggle( cv->vmCvar );
		}

		remapped = ( remapped || cv->teamShader );
	}

	if ( remapped ) {
		G_RemapTeamShaders();
	}

	// check some things
	// DHM - Gametype is currently restricted to supported types only
	trap_Cvar_Set( "g_gametype", va( "%i",GT_WOLF ) );// Nico, set a gametype for queries about server info

	// OSP
	trap_SetConfigstring( CS_SERVERTOGGLES, va( "%d", level.server_settings ) );

	if ( pmove_msec.integer < 8 ) {
		trap_Cvar_Set( "pmove_msec", "8" );
	} else if ( pmove_msec.integer > 33 ) {
		trap_Cvar_Set( "pmove_msec", "33" );
	}

}

/*
=================
G_UpdateCvars
=================
*/
void G_UpdateCvars( void ) {
	int i;
	cvarTable_t *cv;
	qboolean fToggles = qfalse;
	qboolean fVoteFlags = qfalse;
	qboolean remapped = qfalse;
	qboolean chargetimechanged = qfalse;

	for ( i = 0, cv = gameCvarTable ; i < gameCvarTableSize ; i++, cv++ ) {
		if ( cv->vmCvar ) {
			trap_Cvar_Update( cv->vmCvar );

			if ( cv->modificationCount != cv->vmCvar->modificationCount ) {
				cv->modificationCount = cv->vmCvar->modificationCount;

				if ( cv->trackChange && !( cv->cvarFlags & CVAR_LATCH ) ) {
					trap_SendServerCommand( -1, va( "print \"Server:[lof] %s [lon]changed to[lof] %s\n\"", cv->cvarName, cv->vmCvar->string ) );
				}

				if ( cv->teamShader ) {
					remapped = qtrue;
				}

				if ( cv->vmCvar == &g_filtercams ) {
					trap_SetConfigstring( CS_FILTERCAMS, va( "%i", g_filtercams.integer ) );
				}

				if ( cv->vmCvar == &g_soldierChargeTime ) {
					level.soldierChargeTime[0] = g_soldierChargeTime.integer * level.soldierChargeTimeModifier[0];
					level.soldierChargeTime[1] = g_soldierChargeTime.integer * level.soldierChargeTimeModifier[1];
					chargetimechanged = qtrue;
				} else if ( cv->vmCvar == &g_medicChargeTime ) {
					level.medicChargeTime[0] = g_medicChargeTime.integer * level.medicChargeTimeModifier[0];
					level.medicChargeTime[1] = g_medicChargeTime.integer * level.medicChargeTimeModifier[1];
					chargetimechanged = qtrue;
				} else if ( cv->vmCvar == &g_engineerChargeTime ) {
					level.engineerChargeTime[0] = g_engineerChargeTime.integer * level.engineerChargeTimeModifier[0];
					level.engineerChargeTime[1] = g_engineerChargeTime.integer * level.engineerChargeTimeModifier[1];
					chargetimechanged = qtrue;
				} else if ( cv->vmCvar == &g_LTChargeTime ) {
					level.lieutenantChargeTime[0] = g_LTChargeTime.integer * level.lieutenantChargeTimeModifier[0];
					level.lieutenantChargeTime[1] = g_LTChargeTime.integer * level.lieutenantChargeTimeModifier[1];
					chargetimechanged = qtrue;
				} else if ( cv->vmCvar == &g_covertopsChargeTime )    {
					level.covertopsChargeTime[0] = g_covertopsChargeTime.integer * level.covertopsChargeTimeModifier[0];
					level.covertopsChargeTime[1] = g_covertopsChargeTime.integer * level.covertopsChargeTimeModifier[1];
					chargetimechanged = qtrue;
				}
				else if ( cv->vmCvar == &pmove_msec ) {
					if ( pmove_msec.integer < 8 ) {
						trap_Cvar_Set( cv->cvarName, "8" );
					} else if ( pmove_msec.integer > 33 ) {
						trap_Cvar_Set( cv->cvarName, "33" );
					}
				}
				// OSP - Update vote info for clients, if necessary
				if ( cv->vmCvar == &vote_allow_kick          || cv->vmCvar == &vote_allow_map            ||
						cv->vmCvar == &vote_allow_matchreset    ||
						cv->vmCvar == &vote_allow_mutespecs     || cv->vmCvar == &vote_allow_nextmap        ||
						cv->vmCvar == &vote_allow_referee       ||
						cv->vmCvar == &vote_allow_swapteams	 || cv->vmCvar == &vote_allow_antilag        ||
						cv->vmCvar == &vote_allow_muting
						) {
					fVoteFlags = qtrue;
				} else {
					fToggles = ( G_checkServerToggle( cv->vmCvar ) || fToggles );
				}
			}
		}
	}

	if ( fVoteFlags ) {
		G_voteFlags();
	}

	if ( fToggles ) {
		trap_SetConfigstring( CS_SERVERTOGGLES, va( "%d", level.server_settings ) );
	}

	if ( remapped ) {
		G_RemapTeamShaders();
	}

	if ( chargetimechanged ) {
		char cs[MAX_INFO_STRING];
		cs[0] = '\0';
		Info_SetValueForKey( cs, "axs_sld", va( "%i", level.soldierChargeTime[0] ) );
		Info_SetValueForKey( cs, "ald_sld", va( "%i", level.soldierChargeTime[1] ) );
		Info_SetValueForKey( cs, "axs_mdc", va( "%i", level.medicChargeTime[0] ) );
		Info_SetValueForKey( cs, "ald_mdc", va( "%i", level.medicChargeTime[1] ) );
		Info_SetValueForKey( cs, "axs_eng", va( "%i", level.engineerChargeTime[0] ) );
		Info_SetValueForKey( cs, "ald_eng", va( "%i", level.engineerChargeTime[1] ) );
		Info_SetValueForKey( cs, "axs_lnt", va( "%i", level.lieutenantChargeTime[0] ) );
		Info_SetValueForKey( cs, "ald_lnt", va( "%i", level.lieutenantChargeTime[1] ) );
		Info_SetValueForKey( cs, "axs_cvo", va( "%i", level.covertopsChargeTime[0] ) );
		Info_SetValueForKey( cs, "ald_cvo", va( "%i", level.covertopsChargeTime[1] ) );
		trap_SetConfigstring( CS_CHARGETIMES, cs );
	}
}

// Reset particular server variables back to defaults if a config is voted in.
void G_wipeCvars( void ) {
	int i;
	cvarTable_t *pCvars;

	for ( i = 0, pCvars = gameCvarTable; i < gameCvarTableSize; i++, pCvars++ ) {
		if ( pCvars->vmCvar && pCvars->fConfigReset ) {
			G_Printf( "set %s %s\n", pCvars->cvarName, pCvars->defaultString );
			trap_Cvar_Set( pCvars->cvarName, pCvars->defaultString );
		}
	}

	G_UpdateCvars();
}

//bani - #113
#define SNIPSIZE 250

//copies max num chars from beginning of dest into src and returns pointer to new src
char *strcut( char *dest, char *src, int num ) {
	int i;

	if ( !dest || !src || !num ) {
		return NULL;
	}
	for ( i = 0 ; i < num ; i++ ) {
		if ( (char)*src ) {
			*dest = *src;
			dest++;
			src++;
		} else {
			break;
		}
	}
	*dest = (char)0;
	return src;
}

/*
============
G_InitGame

============
*/
void G_InitGame( int levelTime, int randomSeed, int restart ) {
	int i;
	char cs[MAX_INFO_STRING];

	G_Printf( "------- Game Initialization -------\n" );
	G_Printf( "gamename: %s\n", GAME_VERSION );
	G_Printf( "gamedate: %s\n", __DATE__ );

	srand( randomSeed );

	//bani - make sure pak2.pk3 gets referenced on server so pure checks pass
	trap_FS_FOpenFile( "pak2.dat", &i, FS_READ );
	trap_FS_FCloseFile( i );

	G_RegisterCvars();

	G_ProcessIPBans();

	G_InitMemory();

	// NERVE - SMF - intialize gamestate
	if ( g_gamestate.integer == GS_INITIALIZE ) {
		// OSP
		trap_Cvar_Set( "gamestate", va( "%i", GS_PLAYING ) );
	}

	// set some level globals
	i = level.server_settings;
	{
		qboolean oldspawning = level.spawning;
		voteInfo_t votedata;

		memcpy( &votedata, &level.voteInfo, sizeof( voteInfo_t ) );

		memset( &level, 0, sizeof( level ) );

		memcpy( &level.voteInfo, &votedata, sizeof( voteInfo_t ) );

		level.spawning = oldspawning;
	}
	level.time = levelTime;
	level.startTime = levelTime;
	level.server_settings = i;

	for ( i = 0; i < level.numConnectedClients; i++ ) {
		level.clients[ level.sortedClients[ i ] ].sess.spawnObjectiveIndex = 0;
	}

	// RF, init the anim scripting
	level.animScriptData.soundIndex = G_SoundIndex;
	level.animScriptData.playSound = G_AnimScriptSound;

	level.soldierChargeTime[0] = level.soldierChargeTime[1] = g_soldierChargeTime.integer;
	level.medicChargeTime[0] = level.medicChargeTime[1] = g_medicChargeTime.integer;
	level.engineerChargeTime[0] = level.engineerChargeTime[1] = g_engineerChargeTime.integer;
	level.lieutenantChargeTime[0] = level.lieutenantChargeTime[1] = g_LTChargeTime.integer;

	level.covertopsChargeTime[0] = level.covertopsChargeTime[1] = g_covertopsChargeTime.integer;

	level.soldierChargeTimeModifier[0] = level.soldierChargeTimeModifier[1] = 1.f;
	level.medicChargeTimeModifier[0] = level.medicChargeTimeModifier[1] = 1.f;
	level.engineerChargeTimeModifier[0] = level.engineerChargeTimeModifier[1] = 1.f;
	level.lieutenantChargeTimeModifier[0] = level.lieutenantChargeTimeModifier[1] = 1.f;
	level.covertopsChargeTimeModifier[0] = level.covertopsChargeTimeModifier[1] = 1.f;

	cs[0] = '\0';
	Info_SetValueForKey( cs, "axs_sld", va( "%i", level.soldierChargeTime[0] ) );
	Info_SetValueForKey( cs, "ald_sld", va( "%i", level.soldierChargeTime[1] ) );
	Info_SetValueForKey( cs, "axs_mdc", va( "%i", level.medicChargeTime[0] ) );
	Info_SetValueForKey( cs, "ald_mdc", va( "%i", level.medicChargeTime[1] ) );
	Info_SetValueForKey( cs, "axs_eng", va( "%i", level.engineerChargeTime[0] ) );
	Info_SetValueForKey( cs, "ald_eng", va( "%i", level.engineerChargeTime[1] ) );
	Info_SetValueForKey( cs, "axs_lnt", va( "%i", level.lieutenantChargeTime[0] ) );
	Info_SetValueForKey( cs, "ald_lnt", va( "%i", level.lieutenantChargeTime[1] ) );
	Info_SetValueForKey( cs, "axs_cvo", va( "%i", level.covertopsChargeTime[0] ) );
	Info_SetValueForKey( cs, "ald_cvo", va( "%i", level.covertopsChargeTime[1] ) );
	trap_SetConfigstring( CS_CHARGETIMES, cs );
	trap_SetConfigstring( CS_FILTERCAMS, va( "%i", g_filtercams.integer ) );

	G_SoundIndex( "sound/misc/referee.wav"   );
	G_SoundIndex( "sound/misc/vote.wav"      );
	G_SoundIndex( "sound/player/gurp1.wav"   );
	G_SoundIndex( "sound/player/gurp2.wav"   );

	trap_GetServerinfo( cs, sizeof( cs ) );
	Q_strncpyz( level.rawmapname, Info_ValueForKey( cs, "mapname" ), sizeof( level.rawmapname ) );

	trap_SetConfigstring( CS_SCRIPT_MOVER_NAMES, "" );   // clear out

	if ( g_log.string[0] ) {
		if ( g_logSync.integer ) {
			trap_FS_FOpenFile( g_log.string, &level.logFile, FS_APPEND_SYNC );
		} else {
			trap_FS_FOpenFile( g_log.string, &level.logFile, FS_APPEND );
		}
		if ( !level.logFile ) {
			G_Printf( "WARNING: Couldn't open logfile: %s\n", g_log.string );
		} else {
			G_LogPrintf( "------------------------------------------------------------\n" );
			G_LogPrintf( "InitGame: %s\n", cs );
		}
	} else {
		G_Printf( "Not logging to disk.\n" );
	}

	G_InitWorldSession();

	// DHM - Nerve :: Clear out spawn target config strings
	trap_GetConfigstring( CS_MULTI_INFO, cs, sizeof( cs ) );
	Info_SetValueForKey( cs, "numspawntargets", "0" );
	trap_SetConfigstring( CS_MULTI_INFO, cs );

	for ( i = CS_MULTI_SPAWNTARGETS; i < CS_MULTI_SPAWNTARGETS + MAX_MULTI_SPAWNTARGETS; i++ ) {
		trap_SetConfigstring( i, "" );
	}

	G_ResetTeamMapData();

	// initialize all entities for this game
	memset( g_entities, 0, MAX_GENTITIES * sizeof( g_entities[0] ) );
	level.gentities = g_entities;

	// initialize all clients for this game
	level.maxclients = g_maxclients.integer;
	memset( g_clients, 0, MAX_CLIENTS * sizeof( g_clients[0] ) );
	level.clients = g_clients;

	// set client fields on player ents
	for ( i = 0 ; i < level.maxclients ; i++ ) {
		g_entities[i].client = level.clients + i;
	}

	// always leave room for the max number of clients,
	// even if they aren't all used, so numbers inside that
	// range are NEVER anything but clients
	level.num_entities = MAX_CLIENTS;

	// let the server system know where the entites are
	trap_LocateGameData( level.gentities, level.num_entities, sizeof( gentity_t ),
						 &level.clients[0].ps, sizeof( level.clients[0] ) );

	// load level script
	G_Script_ScriptLoad();

	numSplinePaths = 0 ;
	numPathCorners = 0;

	// TAT 11/13/2002
	//		similarly set up the Server entities
	InitServerEntities();

	// parse the key/value pairs and spawn gentities
	G_SpawnEntitiesFromString();

	// TAT 11/13/2002 - entities are spawned, so now we can do setup
	InitialServerEntitySetup();

	// Gordon: debris test
	G_LinkDebris();

	// Gordon: link up damage parents
	G_LinkDamageParents();

	BG_ClearScriptSpeakerPool();

	BG_LoadSpeakerScript( va( "sound/maps/%s.sps", level.rawmapname ) );

	// ===================

	if ( !level.gameManager ) {
		G_Printf( "^1ERROR No 'script_multiplayer' found in map\n" );
	}

	level.tracemapLoaded = qfalse;
	if ( !BG_LoadTraceMap( level.rawmapname, level.mapcoordsMins, level.mapcoordsMaxs ) ) {
		G_Printf( "^1ERROR No tracemap found for map\n" );
	} else {
		level.tracemapLoaded = qtrue;
	}

	// Link all the splines up
	BG_BuildSplinePaths();

	// general initialization
	G_FindTeams();

	G_Printf( "-----------------------------------\n" );

	trap_PbStat( -1, "INIT", "GAME" ) ;

	G_RemapTeamShaders();

	BG_ClearAnimationPool();

	BG_ClearCharacterPool();

	BG_InitWeaponStrings();

	G_RegisterPlayerClasses();

	// Match init work
	G_loadMatchGame();

	// Nico, flood protection
	if (g_floodProtect.integer) {
		if (trap_Cvar_VariableIntegerValue("sv_floodprotect")) {
			trap_Cvar_Set("sv_floodprotect", "0");
		}
	}

	// Nico, is level a timerun?
	if (!level.isTimerun) {
		trap_Cvar_Set("isTimerun", "0");
		G_Printf ("%s: No timerun found in map\n", GAME_VERSION);
	} else {
		trap_Cvar_Set("isTimerun", "1");
	}

	// Nico, load API
	if (g_useAPI.integer) {
		G_loadAPI();
	}
}



/*
=================
G_ShutdownGame
=================
*/
void G_ShutdownGame( int restart ) {
	G_Printf( "==== ShutdownGame ====\n" );

	if ( level.logFile ) {
		G_LogPrintf( "ShutdownGame:\n" );
		G_LogPrintf( "------------------------------------------------------------\n" );
		trap_FS_FCloseFile( level.logFile );
		level.logFile = 0;
	}

	// Nico, unload API
	if (g_useAPI.integer) {
		G_unloadAPI();
	}

	// write all the client session data so we can get it back
	G_WriteSessionData( restart );
}



//===================================================================

#ifndef GAME_HARD_LINKED
// this is only here so the functions in q_shared.c and bg_*.c can link

void QDECL Com_Error( int level, const char *error, ... ) {
	va_list argptr;
	char text[1024];

	va_start( argptr, error );
	Q_vsnprintf( text, sizeof( text ), error, argptr );
	va_end( argptr );

	G_Error( "%s", text );
}
//bani
void QDECL Com_Error( int level, const char *error, ... ) _attribute( ( format( printf,2,3 ) ) );

void QDECL Com_Printf( const char *msg, ... ) {
	va_list argptr;
	char text[1024];

	va_start( argptr, msg );
	Q_vsnprintf( text, sizeof( text ), msg, argptr );
	va_end( argptr );

	G_Printf( "%s", text );
}
//bani
void QDECL Com_Printf( const char *msg, ... ) _attribute( ( format( printf,1,2 ) ) );

#endif

/*
========================================================================

PLAYER COUNTING / SCORE SORTING

========================================================================
*/

/*
=============
SortRanks

=============
*/
int QDECL SortRanks( const void *a, const void *b ) {
	gclient_t   *ca, *cb;

	ca = &level.clients[*(int *)a];
	cb = &level.clients[*(int *)b];

	// sort special clients last
	if ( /*ca->sess.spectatorState == SPECTATOR_SCOREBOARD ||*/ ca->sess.spectatorClient < 0 ) {
		return 1;
	}
	if ( /*cb->sess.spectatorState == SPECTATOR_SCOREBOARD ||*/ cb->sess.spectatorClient < 0  ) {
		return -1;
	}

	// then connecting clients
	if ( ca->pers.connected == CON_CONNECTING ) {
		return 1;
	}
	if ( cb->pers.connected == CON_CONNECTING ) {
		return -1;
	}


	// then spectators
	if ( ca->sess.sessionTeam == TEAM_SPECTATOR && cb->sess.sessionTeam == TEAM_SPECTATOR ) {
		if ( ca->sess.spectatorTime < cb->sess.spectatorTime ) {
			return -1;
		}
		if ( ca->sess.spectatorTime > cb->sess.spectatorTime ) {
			return 1;
		}
		return 0;
	}
	if ( ca->sess.sessionTeam == TEAM_SPECTATOR ) {
		return 1;
	}
	if ( cb->sess.sessionTeam == TEAM_SPECTATOR ) {
		return -1;
	}
	return 0;
}

//bani - #184
//(relatively) sane replacement for OSP's Players_Axis/Players_Allies
void etpro_PlayerInfo( void ) {
	//128 bits
	char playerinfo[MAX_CLIENTS + 1];
	gentity_t *e;
	team_t playerteam;
	int i;
	int lastclient;

	memset( playerinfo, 0, sizeof( playerinfo ) );

	lastclient = -1;
	e = &g_entities[0];
	for ( i = 0; i < MAX_CLIENTS; i++, e++ ) {
		if ( e->client == NULL || e->client->pers.connected == CON_DISCONNECTED ) {
			playerinfo[i] = '-';
			continue;
		}

		//keep track of highest connected/connecting client
		lastclient = i;

		if ( e->inuse == qfalse ) {
			playerteam = 0;
		} else {
			playerteam = e->client->sess.sessionTeam;
		}
		playerinfo[i] = (char)'0' + playerteam;
	}
	//terminate the string, if we have any non-0 clients
	if ( lastclient != -1 ) {
		playerinfo[lastclient + 1] = (char)0;
	} else {
		playerinfo[0] = (char)0;
	}

	trap_Cvar_Set( "P", playerinfo );
}

/*
============
CalculateRanks

Recalculates the score ranks of all players
This will be called on every client connect, begin, disconnect, death,
and team change.
============
*/
void CalculateRanks( void ) {
	int i;
	char teaminfo[TEAM_NUM_TEAMS][256];     // OSP

	level.follow1 = -1;
	level.follow2 = -1;
	level.numConnectedClients = 0;
	level.numNonSpectatorClients = 0;
	level.numPlayingClients = 0;
	level.voteInfo.numVotingClients = 0;        // don't count bots

	level.numFinalDead[0] = 0;      // NERVE - SMF
	level.numFinalDead[1] = 0;      // NERVE - SMF

	level.voteInfo.numVotingTeamClients[ 0 ] = 0;
	level.voteInfo.numVotingTeamClients[ 1 ] = 0;

	for ( i = 0; i < TEAM_NUM_TEAMS; i++ ) {
		if ( i < 2 ) {
			level.numTeamClients[i] = 0;
		}
		teaminfo[i][0] = 0;         // OSP
	}

	for ( i = 0 ; i < level.maxclients ; i++ ) {
		if ( level.clients[i].pers.connected != CON_DISCONNECTED ) {
			int team = level.clients[i].sess.sessionTeam;

			level.sortedClients[level.numConnectedClients] = i;
			level.numConnectedClients++;

			if ( team != TEAM_SPECTATOR ) {
				level.numNonSpectatorClients++;

				// OSP
				Q_strcat( teaminfo[team], sizeof( teaminfo[team] ) - 1, va( "%d ", level.numConnectedClients ) );

				// decide if this should be auto-followed
				if ( level.clients[i].pers.connected == CON_CONNECTED ) {
					int teamIndex = level.clients[i].sess.sessionTeam == TEAM_AXIS ? 0 : 1;
					level.numPlayingClients++;
					if ( !( g_entities[i].r.svFlags & SVF_BOT ) ) {
						level.voteInfo.numVotingClients++;
					}

					if ( level.clients[i].sess.sessionTeam == TEAM_AXIS ||
						 level.clients[i].sess.sessionTeam == TEAM_ALLIES ) {

						level.numTeamClients[teamIndex]++;
						if ( !( g_entities[i].r.svFlags & SVF_BOT ) ) {
							level.voteInfo.numVotingTeamClients[ teamIndex ]++;
						}
					}

					if ( level.follow1 == -1 ) {
						level.follow1 = i;
					} else if ( level.follow2 == -1 ) {
						level.follow2 = i;
					}
				}
			}
		}
	}

	// OSP
	for ( i = 0; i < TEAM_NUM_TEAMS; i++ ) {
		if ( 0 == teaminfo[i][0] ) {
			Q_strncpyz( teaminfo[i], "(None)", sizeof( teaminfo[i] ) );
		}
	}

	qsort( level.sortedClients, level.numConnectedClients,
		   sizeof( level.sortedClients[0] ), SortRanks );

	//bani - #184
	etpro_PlayerInfo();
}


/*
========================================================================

MAP CHANGING

========================================================================
*/

/*
========================
SendScoreboardMessageToAllClients

Do this at BeginIntermission time and whenever ranks are recalculated
due to enters/exits/forced team changes
========================
*/
void SendScoreboardMessageToAllClients( void ) {
	int i;

	for ( i = 0; i < level.numConnectedClients; i++ ) {
		if ( level.clients[level.sortedClients[i]].pers.connected == CON_CONNECTED ) {
			level.clients[level.sortedClients[i]].wantsscore = qtrue;
		}
	}
}

/*
==================
FindIntermissionPoint

This is also used for spectator spawns
==================
*/
void FindIntermissionPoint( void ) {
	gentity_t   *ent = NULL, *target;
	vec3_t dir;
	char cs[MAX_STRING_CHARS];              // DHM - Nerve
	char        *buf;                       // DHM - Nerve
	int winner;                             // DHM - Nerve

	// NERVE - SMF - if the match hasn't ended yet, and we're just a spectator
	// try to find the intermission spawnpoint with no team flags set
	ent = G_Find( NULL, FOFS( classname ), "info_player_intermission" );

	for ( ; ent; ent = G_Find( ent, FOFS( classname ), "info_player_intermission" ) ) {
		if ( !ent->spawnflags ) {
			break;
		}
	}

	trap_GetConfigstring( CS_MULTI_MAPWINNER, cs, sizeof( cs ) );
	buf = Info_ValueForKey( cs, "winner" );
	winner = atoi( buf );

	// Change from scripting value for winner (0==AXIS, 1==ALLIES) to spawnflag value
	if ( winner == 0 ) {
		winner = TEAM_AXIS;
	} else {
		winner = TEAM_ALLIES;
	}


	if ( !ent ) {
		ent = G_Find( NULL, FOFS( classname ), "info_player_intermission" );
		while ( ent ) {
			if ( ent->spawnflags & winner ) {
				break;
			}

			ent = G_Find( ent, FOFS( classname ), "info_player_intermission" );
		}
	}

	if ( !ent ) {    // the map creator forgot to put in an intermission point...
		SelectSpawnPoint( vec3_origin, level.intermission_origin, level.intermission_angle );
	} else {
		VectorCopy( ent->s.origin, level.intermission_origin );
		VectorCopy( ent->s.angles, level.intermission_angle );
		// if it has a target, look towards it
		if ( ent->target ) {
			target = G_PickTarget( ent->target );
			if ( target ) {
				VectorSubtract( target->s.origin, level.intermission_origin, dir );
				vectoangles( dir, level.intermission_angle );
			}
		}
	}

}

/*
=============
ExitLevel

When the intermission has been exited, the server is either killed
or moved to a new level based on the "nextmap" cvar

=============
*/
void ExitLevel( void ) {
	int i;
	gclient_t *cl;

	trap_SendConsoleCommand( EXEC_APPEND, "vstr nextmap\n" );

	level.changemap = NULL;

	for ( i = 0 ; i < g_maxclients.integer ; i++ ) {
		cl = level.clients + i;
		if ( cl->pers.connected != CON_CONNECTED ) {
			continue;
		}
		cl->ps.persistant[PERS_SCORE] = 0;
	}

	// we need to do this here before chaning to CON_CONNECTING
	G_WriteSessionData( qfalse );

	// change all client states to connecting, so the early players into the
	// next level will know the others aren't done reconnecting
	for ( i = 0 ; i < g_maxclients.integer ; i++ ) {

		if ( level.clients[i].pers.connected == CON_CONNECTED ) {
			level.clients[i].pers.connected = CON_CONNECTING;
			trap_UnlinkEntity( &g_entities[i] );
		}
	}

	G_LogPrintf( "ExitLevel: executed\n" );
}

/*
=================
G_LogPrintf

Print to the logfile with a time stamp if it is open
=================
*/
void QDECL G_LogPrintf( const char *fmt, ... ) {
	va_list argptr;
	char string[1024];
	int min, tens, sec, l;

	sec = level.time / 1000;

	min = sec / 60;
	sec -= min * 60;
	tens = sec / 10;
	sec -= tens * 10;

	Com_sprintf( string, sizeof( string ), "%i:%i%i ", min, tens, sec );

	l = strlen( string );

	va_start( argptr, fmt );
	Q_vsnprintf( string + l, sizeof( string ) - l, fmt, argptr );
	va_end( argptr );

	if ( g_dedicated.integer ) {
		G_Printf( "%s", string + l );
	}

	if ( !level.logFile ) {
		return;
	}

	trap_FS_Write( string, strlen( string ), level.logFile );
}
//bani
void QDECL G_LogPrintf( const char *fmt, ... ) _attribute( ( format( printf,1,2 ) ) );

/*
=============
ScoreIsTied
=============
*/
qboolean ScoreIsTied( void ) {
	int a /*, b*/;
	char cs[MAX_STRING_CHARS];
	char    *buf;

	// DHM - Nerve :: GT_WOLF checks the current value of
	trap_GetConfigstring( CS_MULTI_MAPWINNER, cs, sizeof( cs ) );

	buf = Info_ValueForKey( cs, "winner" );
	a = atoi( buf );

	return a == -1;
}

qboolean G_ScriptAction_SetWinner( gentity_t *ent, char *params );

/*
==================
CheckVote
==================
*/
void CheckVote( void ) {
	gentity_t *other = NULL;

	if ( !level.voteInfo.voteTime || level.voteInfo.vote_fn == NULL || level.time - level.voteInfo.voteTime < 1000 ) {
		return;
	}

	// Nico, check if the voter switches teams (from TJMod)
	other = g_entities + level.voteInfo.voter_cn;
	if (level.voteInfo.voter_team != other->client->sess.sessionTeam) {
		AP("cpm \"^5Vote canceled^z: voter switched teams\n\"");
		G_LogPrintf("Vote Failed: %s (voter %s switched teams)\n", level.voteInfo.voteString, other->client->pers.netname);
	} else if ( level.time - level.voteInfo.voteTime >= VOTE_TIME ) {
		AP( va( "cpm \"^2Vote FAILED! ^3(%s)\n\"", level.voteInfo.voteString ) );
		G_LogPrintf( "Vote Failed: %s\n", level.voteInfo.voteString );
	} else {
		int pcnt = ( level.voteInfo.vote_fn == G_StartMatch_v ) ? 75 : vote_percent.integer;
		int total;

		if ( pcnt > 99 ) {
			pcnt = 99;
		}
		if ( pcnt < 1 ) {
			pcnt = 1;
		}

		if ( level.voteInfo.vote_fn == G_Kick_v ) {
			gentity_t* other = &g_entities[ atoi( level.voteInfo.vote_value ) ];
			if ( !other->client || other->client->sess.sessionTeam == TEAM_SPECTATOR ) {
				total = level.voteInfo.numVotingClients;
			} else {
				total = level.voteInfo.numVotingTeamClients[ other->client->sess.sessionTeam == TEAM_AXIS ? 0 : 1 ];
			}
		} else {
			total = level.voteInfo.numVotingClients;
		}

		if ( level.voteInfo.voteYes > pcnt * total / 100 ) {
			// execute the command, then remove the vote
			if ( level.voteInfo.voteYes > total + 1 ) {
				// Don't announce some votes, as in comp mode, it is generally a ref
				// who is policing people who shouldn't be joining and players don't want
				// this sort of spam in the console
				if ( level.voteInfo.vote_fn != G_Kick_v ) {
					AP( va( "cpm \"^5Referee changed setting! ^7(%s)\n\"", level.voteInfo.voteString ) );
				}
				G_LogPrintf( "Referee Setting: %s\n", level.voteInfo.voteString );
			} else {
				AP( "cpm \"^5Vote passed!\n\"" );
				G_LogPrintf( "Vote Passed: %s\n", level.voteInfo.voteString );
			}

			// Perform the passed vote
			level.voteInfo.vote_fn( NULL, 0, NULL, NULL, qfalse );

		} else if ( level.voteInfo.voteNo && level.voteInfo.voteNo >= ( 100 - pcnt ) * total / 100 ) {
			// same behavior as a no response vote
			AP( va( "cpm \"^2Vote FAILED! ^3(%s)\n\"", level.voteInfo.voteString ) );
			G_LogPrintf( "Vote Failed: %s\n", level.voteInfo.voteString );
		} else {
			// still waiting for a majority
			return;
		}
	}

	level.voteInfo.voteTime = 0;
	trap_SetConfigstring( CS_VOTE_TIME, "" );
}

/*
==================
CheckCvars
==================
*/
void CheckCvars( void ) {
	static int g_password_lastMod = -1;

	if ( g_password.modificationCount != g_password_lastMod ) {
		g_password_lastMod = g_password.modificationCount;
		if ( *g_password.string && Q_stricmp( g_password.string, "none" ) ) {
			trap_Cvar_Set( "g_needpass", "1" );
		} else {
			trap_Cvar_Set( "g_needpass", "0" );
		}
	}
}

/*
=============
G_RunThink

Runs thinking code for this frame if necessary
=============
*/
void G_RunThink( gentity_t *ent ) {
	float thinktime;

	// OSP - If paused, push nextthink
	if ( level.match_pause != PAUSE_NONE && ( ent - g_entities ) >= g_maxclients.integer &&
		 ent->nextthink > level.time && strstr( ent->classname, "DPRINTF_" ) == NULL ) {
		ent->nextthink += level.time - level.previousTime;
	}

	// RF, run scripting
	if ( ent->s.number >= MAX_CLIENTS ) {
		G_Script_ScriptRun( ent );
	}

	thinktime = ent->nextthink;
	if ( thinktime <= 0 ) {
		return;
	}
	if ( thinktime > level.time ) {
		return;
	}

	ent->nextthink = 0;
	if ( !ent->think ) {
		G_Error( "NULL ent->think" );
	}
	ent->think( ent );
}

void G_RunEntity( gentity_t* ent, int msec );

/*
======================
G_PositionEntityOnTag
======================
*/
qboolean G_PositionEntityOnTag( gentity_t *entity, gentity_t* parent, char *tagName ) {
	int i;
	orientation_t tag;
	vec3_t axis[3];
	AnglesToAxis( parent->r.currentAngles, axis );

	VectorCopy( parent->r.currentOrigin, entity->r.currentOrigin );

	if ( !trap_GetTag( -1, parent->tagNumber, tagName, &tag ) ) {
		return qfalse;
	}

	for ( i = 0 ; i < 3 ; i++ ) {
		VectorMA( entity->r.currentOrigin, tag.origin[i], axis[i], entity->r.currentOrigin );
	}

	if ( entity->client && entity->s.eFlags & EF_MOUNTEDTANK ) {
		// zinx - moved tank hack to here
		// bani - fix tank bb
		// zinx - figured out real values, only tag_player is applied,
		// so there are two left:
		// mg42upper attaches to tag_mg42nest[mg42base] at:
		// 0.03125, -1.171875, 27.984375
		// player attaches to tag_playerpo[mg42upper] at:
		// 3.265625, -1.359375, 2.96875
		// this is a hack, by the way.
		entity->r.currentOrigin[0] += 0.03125 + 3.265625;
		entity->r.currentOrigin[1] += -1.171875 + - 1.359375;
		entity->r.currentOrigin[2] += 27.984375 + 2.96875;
	}

	G_SetOrigin( entity, entity->r.currentOrigin );

	if ( entity->r.linked && !entity->client ) {
		if ( !VectorCompare( entity->oldOrigin, entity->r.currentOrigin ) ) {
			trap_LinkEntity( entity );
		}
	}

	return qtrue;
}

void G_TagLinkEntity( gentity_t* ent, int msec ) {
	gentity_t* parent = &g_entities[ent->s.torsoAnim];
	vec3_t move, amove;
	gentity_t* obstacle;
	vec3_t origin, angles;
	vec3_t v;

	if ( ent->linkTagTime >= level.time ) {
		return;
	}

	G_RunEntity( parent, msec );

	if ( !( parent->s.eFlags & EF_PATH_LINK ) ) {
		if ( parent->s.pos.trType == TR_LINEAR_PATH ) {
			int pos;
			float frac;

			if ( ( ent->backspline = BG_GetSplineData( parent->s.effect2Time, &ent->back ) ) == NULL ) {
				return;
			}

			ent->backdelta = parent->s.pos.trDuration ? ( level.time - parent->s.pos.trTime ) / ( (float)parent->s.pos.trDuration ) : 0;

			if ( ent->backdelta < 0.f ) {
				ent->backdelta = 0.f;
			} else if ( ent->backdelta > 1.f ) {
				ent->backdelta = 1.f;
			}

			if ( ent->back ) {
				ent->backdelta = 1 - ent->backdelta;
			}

			pos = floor( ent->backdelta * ( MAX_SPLINE_SEGMENTS ) );
			if ( pos >= MAX_SPLINE_SEGMENTS ) {
				pos = MAX_SPLINE_SEGMENTS - 1;
				frac = ent->backspline->segments[pos].length;
			} else {
				frac = ( ( ent->backdelta * ( MAX_SPLINE_SEGMENTS ) ) - pos ) * ent->backspline->segments[pos].length;
			}


			VectorMA( ent->backspline->segments[pos].start, frac, ent->backspline->segments[pos].v_norm, v );
			if ( parent->s.apos.trBase[0] ) {
				BG_LinearPathOrigin2( parent->s.apos.trBase[0], &ent->backspline, &ent->backdelta, v, ent->back );
			}

			VectorCopy( v, origin );

			if ( ent->s.angles2[0] ) {
				BG_LinearPathOrigin2( ent->s.angles2[0], &ent->backspline, &ent->backdelta, v, ent->back );
			}

			VectorCopy( v, ent->backorigin );

			if ( ent->s.angles2[0] < 0 ) {
				VectorSubtract( v, origin, v );
				vectoangles( v, angles );
			} else if ( ent->s.angles2[0] > 0 ) {
				VectorSubtract( origin, v, v );
				vectoangles( v, angles );
			} else {
				VectorCopy( vec3_origin, origin );
			}

			ent->moving = qtrue;
		} else {
			ent->moving = qfalse;
		}
	} else {
		if ( parent->moving ) {
			VectorCopy( parent->backorigin, v );

			ent->back =         parent->back;
			ent->backdelta =    parent->backdelta;
			ent->backspline =   parent->backspline;

			VectorCopy( v, origin );

			if ( ent->s.angles2[0] ) {
				BG_LinearPathOrigin2( ent->s.angles2[0], &ent->backspline, &ent->backdelta, v, ent->back );
			}

			VectorCopy( v, ent->backorigin );

			if ( ent->s.angles2[0] < 0 ) {
				VectorSubtract( v, origin, v );
				vectoangles( v, angles );
			} else if ( ent->s.angles2[0] > 0 ) {
				VectorSubtract( origin, v, v );
				vectoangles( v, angles );
			} else {
				VectorCopy( vec3_origin, origin );
			}

			ent->moving = qtrue;
		} else {
			ent->moving = qfalse;
		}
	}

	if ( ent->moving ) {
		VectorSubtract( origin, ent->r.currentOrigin, move );
		VectorSubtract( angles, ent->r.currentAngles, amove );

		if ( !G_MoverPush( ent, move, amove, &obstacle ) ) {
			script_mover_blocked( ent, obstacle );
		}

		VectorCopy( origin, ent->s.pos.trBase );
		VectorCopy( angles, ent->s.apos.trBase );
	} else {
		memset( &ent->s.pos, 0, sizeof( ent->s.pos ) );
		memset( &ent->s.apos, 0, sizeof( ent->s.apos ) );

		VectorCopy( ent->r.currentOrigin, ent->s.pos.trBase );
		VectorCopy( ent->r.currentAngles, ent->s.apos.trBase );
	}

	ent->linkTagTime = level.time;
}

void G_RunEntity( gentity_t* ent, int msec ) {
	if ( ent->runthisframe ) {
		return;
	}

	ent->runthisframe = qtrue;

	if ( !ent->inuse ) {
		return;
	}

	if ( ent->tagParent ) {

		G_RunEntity( ent->tagParent, msec );

		if ( ent->tagParent ) {
			if ( G_PositionEntityOnTag( ent, ent->tagParent, ent->tagName ) ) {
				if ( !ent->client ) {
					if ( !ent->s.density ) {
						BG_EvaluateTrajectory( &ent->s.apos, level.time, ent->r.currentAngles, qtrue, ent->s.effect2Time  );
						VectorAdd( ent->tagParent->r.currentAngles, ent->r.currentAngles, ent->r.currentAngles );
					} else {
						BG_EvaluateTrajectory( &ent->s.apos, level.time, ent->r.currentAngles, qtrue, ent->s.effect2Time  );
					}
				}
			}
		}
	} else if ( ent->s.eFlags & EF_PATH_LINK ) {

		G_TagLinkEntity( ent, msec );
	}

	// ydnar: hack for instantaneous velocity
	VectorCopy( ent->r.currentOrigin, ent->oldOrigin );

	// check EF_NODRAW status for non-clients
	if ( ent - g_entities > level.maxclients ) {
		if ( ent->flags & FL_NODRAW ) {
			ent->s.eFlags |= EF_NODRAW;
		} else {
			ent->s.eFlags &= ~EF_NODRAW;
		}
	}


	// clear events that are too old
	if ( level.time - ent->eventTime > EVENT_VALID_MSEC ) {
		if ( ent->s.event ) {
			ent->s.event = 0;
		}
		if ( ent->freeAfterEvent ) {
			// tempEntities or dropped items completely go away after their event
			G_FreeEntity( ent );
			return;
		} else if ( ent->unlinkAfterEvent ) {
			// items that will respawn will hide themselves after their pickup event
			ent->unlinkAfterEvent = qfalse;
			trap_UnlinkEntity( ent );
		}
	}

	// temporary entities don't think
	if ( ent->freeAfterEvent ) {
		return;
	}

	// Arnout: invisible entities don't think
	// NOTE: hack - constructible one does
	if ( ent->s.eType != ET_CONSTRUCTIBLE && ( ent->entstate == STATE_INVISIBLE || ent->entstate == STATE_UNDERCONSTRUCTION ) ) {
		// Gordon: we want them still to run scripts tho :p
		if ( ent->s.number >= MAX_CLIENTS ) {
			G_Script_ScriptRun( ent );
		}
		return;
	}

	if ( !ent->r.linked && ent->neverFree ) {
		return;
	}

	if ( ent->s.eType == ET_MISSILE
		 || ent->s.eType == ET_FLAMEBARREL
		 || ent->s.eType == ET_FP_PARTS
		 || ent->s.eType == ET_FIRE_COLUMN
		 || ent->s.eType == ET_FIRE_COLUMN_SMOKE
		 || ent->s.eType == ET_EXPLO_PART
		 || ent->s.eType == ET_RAMJET ) {

		// OSP - pausing
		if ( level.match_pause == PAUSE_NONE ) {
			G_RunMissile( ent );
		} else {
			// During a pause, gotta keep track of stuff in the air
			ent->s.pos.trTime += level.time - level.previousTime;
			// Keep pulsing right for dynmamite
			if ( ent->methodOfDeath == MOD_DYNAMITE ) {
				ent->s.effect1Time += level.time - level.previousTime;
			}
			G_RunThink( ent );
		}
		// OSP

		return;
	}

	// DHM - Nerve :: Server-side collision for flamethrower
	if ( ent->s.eType == ET_FLAMETHROWER_CHUNK ) {
		G_RunFlamechunk( ent );

		// ydnar: hack for instantaneous velocity
		VectorSubtract( ent->r.currentOrigin, ent->oldOrigin, ent->instantVelocity );
		VectorScale( ent->instantVelocity, 1000.0f / msec, ent->instantVelocity );

		return;
	}

	if ( ent->s.eType == ET_ITEM || ent->physicsObject ) {
		G_RunItem( ent );

		// ydnar: hack for instantaneous velocity
		VectorSubtract( ent->r.currentOrigin, ent->oldOrigin, ent->instantVelocity );
		VectorScale( ent->instantVelocity, 1000.0f / msec, ent->instantVelocity );

		return;
	}

	if ( ent->s.eType == ET_MOVER || ent->s.eType == ET_PROP ) {
		G_RunMover( ent );

		// ydnar: hack for instantaneous velocity
		VectorSubtract( ent->r.currentOrigin, ent->oldOrigin, ent->instantVelocity );
		VectorScale( ent->instantVelocity, 1000.0f / msec, ent->instantVelocity );

		return;
	}

	if ( ent - g_entities < MAX_CLIENTS ) {
		G_RunClient( ent );

		// ydnar: hack for instantaneous velocity
		VectorSubtract( ent->r.currentOrigin, ent->oldOrigin, ent->instantVelocity );
		VectorScale( ent->instantVelocity, 1000.0f / msec, ent->instantVelocity );

		return;
	}

	if ( ( ent->s.eType == ET_HEALER || ent->s.eType == ET_SUPPLIER ) && ent->target_ent ) {
		ent->target_ent->s.onFireStart =    ent->health;
		ent->target_ent->s.onFireEnd =      ent->count;
	}

	G_RunThink( ent );

	// ydnar: hack for instantaneous velocity
	VectorSubtract( ent->r.currentOrigin, ent->oldOrigin, ent->instantVelocity );
	VectorScale( ent->instantVelocity, 1000.0f / msec, ent->instantVelocity );
}

/*
================
G_RunFrame

Advances the non-player objects in the world
================
*/
void G_RunFrame( int levelTime ) {
	int i, msec;


	// if we are waiting for the level to restart, do nothing
	if ( level.restarted ) {
		return;
	}

	// Handling of pause offsets
	if ( level.match_pause == PAUSE_NONE ) {
		level.timeCurrent = levelTime - level.timeDelta;
	} else {
		level.timeDelta = levelTime - level.timeCurrent;
		if ( ( level.time % 500 ) == 0 ) {
			// FIXME: set a PAUSE cs and let the client adjust their local starttimes
			//        instead of this spam
			trap_SetConfigstring( CS_LEVEL_START_TIME, va( "%i", level.startTime + level.timeDelta ) );
		}
	}

	level.frameTime = trap_Milliseconds();

	level.framenum++;
	level.previousTime = level.time;
	level.time = levelTime;

	msec = level.time - level.previousTime;

	level.axisBombCounter -= msec;
	level.alliedBombCounter -= msec;

	if ( level.axisBombCounter < 0 ) {
		level.axisBombCounter = 0;
	}
	if ( level.alliedBombCounter < 0 ) {
		level.alliedBombCounter = 0;
	}

	// get any cvar changes
	G_UpdateCvars();

	for ( i = 0; i < level.num_entities; i++ ) {
		g_entities[i].runthisframe = qfalse;
	}

	// go through all allocated objects
	for ( i = 0; i < level.num_entities; i++ ) {
		G_RunEntity( &g_entities[ i ], msec );
	}


	for ( i = 0; i < level.numConnectedClients; i++ ) {
		ClientEndFrame( &g_entities[level.sortedClients[i]] );
	}

	// update to team status?
	CheckTeamStatus();

	// cancel vote if timed out
	CheckVote();

	// Nico, check for delayed map change
	G_check_delayed_map_change();

	// for tracking changes
	CheckCvars();

	G_UpdateTeamMapData();
}
