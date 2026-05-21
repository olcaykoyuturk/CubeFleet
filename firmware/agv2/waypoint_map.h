#pragma once
#include "types.h"   // Heading enum için

// =============================================================================
// waypoint_map.h — AGV grid haritası (derleme zamanında sabit)
//
// Grid yapısı (10 kenar, cyclic — alternatif rota mümkün):
//   A─B─C
//     │ │
//   D─E F
//   │ │ │
//   G─H─I
//
// KUZEY = yukarı (A,B,C yönü)   GÜNEY = aşağı (G,H,I yönü)
// DOĞU  = sağ   (C,F,I yönü)   BATI  = sol   (A,D,G yönü)
//
// 2026-05 güncelleme: G-H ve C-F kenarları eklendi (graf'ı ağaçtan döngülü
// hale getirdi; multi-AGV alternatif rota artık mümkün).
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
    //  İsim   Komşular                                                          Adet
    {'A', {{'B', EAST}},                                                          1},
    {'B', {{'A', WEST},  {'C', EAST},  {'E', SOUTH}},                             3},
    {'C', {{'B', WEST},  {'F', SOUTH}},                                           2},
    {'D', {{'E', EAST},  {'G', SOUTH}},                                           2},
    {'E', {{'D', WEST},  {'B', NORTH}, {'H', SOUTH}},                             3},
    {'F', {{'I', SOUTH}, {'C', NORTH}},                                           2},
    {'G', {{'D', NORTH}, {'H', EAST}},                                            2},
    {'H', {{'E', NORTH}, {'I', EAST},  {'G', WEST}},                              3},
    {'I', {{'H', WEST},  {'F', NORTH}},                                           2},
};
