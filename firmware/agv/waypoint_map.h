#pragma once
#include "types.h"   // Heading enum için

// =============================================================================
// waypoint_map.h — AGV grid haritası (derleme zamanında sabit)
//
// Grid yapısı (12 kenar, 4×3 — A-L):
//   A─B─C─D
//   │   │
//   E─F─G─H
//       │
//   I─J─K─L
//
// KUZEY = yukarı (A,B,C,D yönü)   GÜNEY = aşağı (I,J,K,L yönü)
// DOĞU  = sağ   (D,H,L yönü)      BATI  = sol   (A,E,I yönü)
//
// 2026-06: 3×3 (A-I) düzeninden 4×3 (A-L) yeni saha düzenine geçildi.
// Dikey bağlantılar yalnızca A-E, C-G, F-J (sağ sütun D-H-L bağlı değil).
// =============================================================================

#define NUM_WAYPOINTS   12
#define MAX_NEIGHBORS   4

struct Connection {
    char    to;    // komşu waypoint adı ('A'–'L')
    Heading dir;   // bu yönde gidilince o komşuya ulaşılır
};

struct WaypointDef {
    char       name;
    Connection neighbors[MAX_NEIGHBORS];
    uint8_t    numNeighbors;
};

static const WaypointDef WAYPOINT_MAP[NUM_WAYPOINTS] = {
    //  İsim   Komşular                                              Adet
    {'A', {{'B', EAST},  {'E', SOUTH}},                              2},
    {'B', {{'A', WEST},  {'C', EAST}},                               2},
    {'C', {{'B', WEST},  {'D', EAST},  {'G', SOUTH}},                3},
    {'D', {{'C', WEST}},                                             1},
    {'E', {{'A', NORTH}, {'F', EAST}},                               2},
    {'F', {{'E', WEST},  {'G', EAST},  {'J', SOUTH}},                3},
    {'G', {{'C', NORTH}, {'F', WEST},  {'H', EAST}},                 3},
    {'H', {{'G', WEST}},                                             1},
    {'I', {{'J', EAST}},                                             1},
    {'J', {{'F', NORTH}, {'I', WEST},  {'K', EAST}},                 3},
    {'K', {{'J', WEST},  {'L', EAST}},                               2},
    {'L', {{'K', WEST}},                                             1},
};
