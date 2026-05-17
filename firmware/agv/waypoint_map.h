#pragma once
#include "types.h"   // Heading enum için

// =============================================================================
// waypoint_map.h — AGV grid haritası (derleme zamanında sabit)
//
// Grid yapısı:
//   A─B─C
//     │
//   D─E   F
//   │ │   │
//   G H───I
//
// KUZEY = yukarı (A,B,C yönü)   GÜNEY = aşağı (G,H,I yönü)
// DOĞU  = sağ   (C,F,I yönü)   BATI  = sol   (A,D,G yönü)
// =============================================================================

#define NUM_WAYPOINTS   9
#define MAX_NEIGHBORS   4

struct Connection {
    char    to;    // komşu waypoint adı ('A'–'I')
    Heading dir;   // bu yönde gidilince o komşuya ulaşılır
};

struct WaypointDef {
    char       name;
    Connection neighbors[MAX_NEIGHBORS];
    uint8_t    numNeighbors;
};

static const WaypointDef WAYPOINT_MAP[NUM_WAYPOINTS] = {
    //  İsim   Komşular                                              Adet
    {'A', {{'B', EAST}},                                             1},
    {'B', {{'A', WEST}, {'C', EAST}, {'E', SOUTH}},                  3},
    {'C', {{'B', WEST}},                                             1},
    {'D', {{'E', EAST}, {'G', SOUTH}},                               2},
    {'E', {{'D', WEST}, {'B', NORTH}, {'H', SOUTH}},                 3},
    {'F', {{'I', SOUTH}},                                            1},
    {'G', {{'D', NORTH}},                                            1},
    {'H', {{'E', NORTH}, {'I', EAST}},                               2},
    {'I', {{'H', WEST}, {'F', NORTH}},                               2},
};
