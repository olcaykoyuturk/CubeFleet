// ===== Yol Bulma Modülü =====
// Path planning artik PC'de yapiliyor (A* + multi-AGV çakışma yönetimi).
// AGV firmware sadece harita lookup ile sınırlı: junction'da hangi yöne
// dönüleceğini bilmesi yeterli. BFS findPath fonksiyonu kaldırıldı —
// re-path PC'nin reroute kararıyla çakışıyordu.

#include "types.h"
#include "waypoint_map.h"

// WAYPOINT_MAP içinde 'name' için indeks döndürür, bulunamazsa -1.
static int wpIndex(char name) {
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
        if (WAYPOINT_MAP[i].name == name) return i;
    }
    return -1;
}

// 'from' waypoint'inden komşu 'to'ya gitmek için fiziksel yönü döndürür.
// Junction'da motor donus yonunu hesaplamak icin kullanilir.
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
