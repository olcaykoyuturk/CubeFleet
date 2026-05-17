// ===== Yol Bulma Modülü =====
// BFS ile iki waypoint arasında en kısa yolu bulur.
// getDirection() ile iki komşu arasındaki fiziksel yön sorgulanır.

#include "types.h"
#include "waypoint_map.h"

// WAYPOINT_MAP içinde 'name' için indeks döndürür, bulunamazsa -1.
static int wpIndex(char name) {
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
        if (WAYPOINT_MAP[i].name == name) return i;
    }
    return -1;
}

// BFS: 'from'dan 'to'ya en kısa yol.
// path[] dizisini waypoint isimleriyle doldurur, *pathLength'e uzunluğu yazar.
// Başarılıysa true, yol bulunamazsa false döner.
bool findPath(char from, char to, char* path, int* pathLength) {
    if (from == 0 || to == 0)   return false;
    if (wpIndex(from) < 0 || wpIndex(to) < 0) return false;

    if (from == to) {
        path[0]     = from;
        *pathLength = 1;
        return true;
    }

    char parent[NUM_WAYPOINTS];
    bool visited[NUM_WAYPOINTS];
    char queue[NUM_WAYPOINTS];
    int  qHead = 0, qTail = 0;

    for (int i = 0; i < NUM_WAYPOINTS; i++) {
        parent[i]  = 0;
        visited[i] = false;
    }

    int fromIdx = wpIndex(from);
    visited[fromIdx] = true;
    parent[fromIdx]  = from;   // kök: kendisine işaret eder
    queue[qTail++]   = from;

    bool found = false;
    while (qHead < qTail && !found) {
        char cur    = queue[qHead++];
        int  curIdx = wpIndex(cur);
        const WaypointDef& wp = WAYPOINT_MAP[curIdx];

        for (int i = 0; i < wp.numNeighbors; i++) {
            char nb    = wp.neighbors[i].to;
            int  nbIdx = wpIndex(nb);
            if (nbIdx < 0 || visited[nbIdx]) continue;
            visited[nbIdx] = true;
            parent[nbIdx]  = cur;
            queue[qTail++] = nb;
            if (nb == to) { found = true; break; }
        }
    }

    if (!found) return false;

    // Yolu geriden izle
    char temp[NUM_WAYPOINTS];
    int  len = 0;
    char cur = to;
    while (cur != from && len <= NUM_WAYPOINTS) {
        temp[len++] = cur;
        cur = parent[wpIndex(cur)];
    }
    temp[len++] = from;

    // Ters çevir → path[]
    *pathLength = len;
    for (int i = 0; i < len; i++) path[i] = temp[len - 1 - i];
    return true;
}

// 'from' waypoint'inden komşu 'to'ya gitmek için fiziksel yönü döndürür.
bool getDirection(char from, char to, Heading* dir) {
    int idx = wpIndex(from);
    if (idx < 0) return false;
    const WaypointDef& wp = WAYPOINT_MAP[idx];
    for (int i = 0; i < wp.numNeighbors; i++) {
        if (wp.neighbors[i].to == to) {
            *dir = wp.neighbors[i].dir;
            return true;
        }
    }
    return false;
}
