#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <limits>
#include <cmath>
#include <cstdint>
#include <string>
#include <tuple>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <map>
#include <windows.h>

// Konverzija stepeni u radijane
double toRadians(double degree) {
    return degree * M_PI / 180.0;
}

// Funkcija vraća rastojanje u kilometrima
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // Poluprečnik Zemlje u kilometrima 
    double R = 6371.0;

    double dLat = toRadians(lat2 - lat1);
    double dLon = toRadians(lon2 - lon1);

    lat1 = toRadians(lat1);
    lat2 = toRadians(lat2);

    // Haversine formula
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) *
               sin(dLon / 2) * sin(dLon / 2);
               
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c; // Udaljenost u km
}

// Pomoćna funkcija za parsiranje CSV linije 
std::vector<std::string> parseCSVLine(const std::string& line) {
    std::vector<std::string> result;
    std::string current = "";
    bool inQuotes = false;
    
    for (size_t i = 0; i < line.length(); ++i) {
        char c = line[i];
        if (c == '"') {
            inQuotes = !inQuotes;
            // Preskačemo same navodnike da ne bi završili u stringu
            continue; 
        } else if (c == ',' && !inQuotes) {
            result.push_back(current);
            current = "";
        } else {
            current += c;
        }
    }
    result.push_back(current);
    return result;
}

// Konverzija stringa vremena "00:01:00" u sekunde ili minute (double)
double parseTimeToSeconds(const std::string& timeStr) {
    int h = 0, m = 0, s = 0;
    char colon;
    std::stringstream ss(timeStr);
    ss >> h >> colon >> m >> colon >> s;
    return h * 3600.0 + m * 60.0 + s; 
}

typedef struct Station
{
    std::string id;
    std::string name;

    double latitude;
    double longitude;

    uint64_t lineMask;
} Station;

typedef struct Route
{
    std::string id;
    std::string shortName;
    std::string longName;
} Route;

struct ParentInfo {
    int parent_id = -1;
    uint64_t line_mask = 0;
};

class Graph {
    public:
        Graph(
            const std::string& routesFile, 
            const std::string& stopsFile, 
            const std::string& edgesFile
        ) {

            std::ifstream file(routesFile);
            if (!file.is_open()) {
                throw std::runtime_error("Greška: Nije moguće otvoriti fajl -> " + routesFile);
            }

            std::string line;
            std::getline(file, line); // Preskakanje zaglavlja (header-a)

            // Čitanje liniju po liniju u petlji
            int i = 0;
            while (std::getline(file, line)) {
                auto row = parseCSVLine(line);
                routes.push_back(Route{row[0], row[1], row[2]});
                routesToIndex[row[0]] = i;
                i++;
            }
            file.close();
            
            file.open(stopsFile);
            if (!file.is_open()) {
                throw std::runtime_error("Greška: Nije moguće otvoriti fajl -> " + stopsFile);
            }

            std::getline(file, line); // Preskakanje zaglavlja (header-a)
            i = 0;
            while (std::getline(file, line)) {
                auto row = parseCSVLine(line);
                stations.push_back(Station{row[0], row[1], std::stod(row[2]), std::stod(row[3]), 0});
                stationsToIndex[row[0]] = i;
                nameToIndex[row[1]] = i; // Dodajemo mapiranje imena stanice na njen indeks
                i++;
            }
            file.close();

            this->V = stations.size();
            adjacency_list.resize(V);
            parents.resize(V, {-1, 0}); // Inicijalizacija roditeljskog niza

            file.open(edgesFile);
            if (!file.is_open()) {
                throw std::runtime_error("Greška: Nije moguće otvoriti fajl -> " + edgesFile);
            }   

            std::getline(file, line); // Preskakanje zaglavlja (header-a)
            while (std::getline(file, line)) {
                auto row = parseCSVLine(line);
                int u = stationsToIndex[row[1]];
                int v = stationsToIndex[row[2]];
                double weight = parseTimeToSeconds(row[3]); // Pretvaramo vreme u sekunde
                addEdge(u, v, weight);
                stations[u].lineMask |= (1ULL << routesToIndex[row[0]]);
                stations[v].lineMask |= (1ULL << routesToIndex[row[0]]);
            }
            file.close();
        }

        int getV() {
            return V;
        }

        std::unordered_map<std::string, int> getNameToIndexMap() {
            return nameToIndex;
        }

        void addEdge(int u, int v, double weight) {
            adjacency_list[u].emplace_back(v, weight);
            adjacency_list[v].emplace_back(u, weight);
        }

        void setStation(int index, const Station& station) {
            stations[index] = station;
        }

        double heuristic(int node, int goal) {
            double lat1 = stations[node].latitude;
            double lon1 = stations[node].longitude;
            double lat2 = stations[goal].latitude;
            double lon2 = stations[goal].longitude;

            return calculateDistance(lat1, lon1, lat2, lon2) / 32.5 * 3600; // Pretvaramo u sekunde, pretpostavljajući prosečnu brzinu metroa od 32.5 km/h
        }

        void AStar(int start, int goal) {
            auto cmp = [](const std::tuple<int, double, uint64_t>& a, const std::tuple<int, double, uint64_t>& b) {
                return std::get<1>(a) > std::get<1>(b); // Min-heap po drugom elementu (f)
            }; 

            std::priority_queue<
                std::tuple<int, double, uint64_t>, 
                std::vector<std::tuple<int, double, uint64_t>>, 
                decltype(cmp)
            > open_list(cmp);
            // int je broj cvora (n), double je f(n) a uint64_t je bitmask linija kojima sam dosla u cvor n

            std::vector<bool> in_closed_list(V, false);
            std::vector<double> g(V, std::numeric_limits<double>::infinity());

            // Resetujemo parents niz za svako novo pokretanje pretrage
            for(int i = 0; i < V; ++i) {
                parents[i] = {-1, 0};
            }

            // Inicijalizacija početnog čvora
            g[start] = 0;
            uint64_t initialLineMask = stations[start].lineMask; // Pretpostavljamo da je lineMask definisan za početnu stanicu
            open_list.emplace(start, g[start] + heuristic(start, goal), initialLineMask);

            std::tuple<int, double, uint64_t> tmp;

            while (!open_list.empty()) {
                tmp = open_list.top();
                open_list.pop();

                int n = std::get<0>(tmp);
                uint64_t currentLineMask = std::get<2>(tmp);

                // Ako je čvor već zatvoren, ignorišemo ovaj stari/duplirani zapis iz heapa
                if (in_closed_list[n]) {
                    continue;
                }

                // Ako smo stigli do cilja
                if (n == goal) {
                    double minutes = g[n] / 60.0; // Pretvaramo sekunde u minute
                    std::cout << "Put je pronadjen! Ukupna cena: " << minutes << " minuta" << std::endl;
                    reconstructPath(start, goal);
                    return;
                }

                // Prebacujemo cvor u zatvorenu listu
                in_closed_list[n] = true;

                // Prolazak kroz sve susede
                for (const std::pair<int, double>& neighbour : adjacency_list[n]) {
                    int m = neighbour.first;
                    double weight = neighbour.second;
                    uint64_t neighbourLineMask = stations[m].lineMask;

                    uint64_t commonLines = currentLineMask & neighbourLineMask;

                    if (commonLines == 0) {
                        commonLines = stations[n].lineMask & neighbourLineMask;
                    }

                    double tentative_g = g[n] + weight;

                    // Ako smo našli kraći put do suseda
                    if (tentative_g < g[m]) {
                        g[m] = tentative_g;
                        parents[m] = {n, commonLines}; // Čuvamo roditelja i masku linije zbog rekonstrukcije puta
                        
                        open_list.emplace(m, g[m] + heuristic(m, goal), commonLines);

                        if (in_closed_list[m]) {
                            in_closed_list[m] = false; // Ponovo otvaramo čvor ako je bio zatvoren
                        }
                    }
                }
            }

            std::cout << "Put nije pronadjen!" << std::endl;
        }

        void reconstructPath(int start, int goal) {
            std::vector<int> path;
            int current = goal;

            // Sakupljamo putanju unazad
            while (current != start) {
                path.push_back(current);
                current = parents[current].parent_id;
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end()); // Sada imamo tačan redosled stanica: A -> B -> C ...

            
            if (path.size() < 2) return;

            // Uzimamo sve zajedničke linije na prvom koraku
            uint64_t activeLines = parents[path[1]].line_mask;
            int segmentStartIdx = 0;

            for (size_t i = 1; i < path.size() - 1; ++i) {
                int u = path[i];
                int v = path[i + 1];

                // Koje linije povezuju sledeći par stanica (u -> v)?
                uint64_t nextStepLines = stations[u].lineMask & stations[v].lineMask;

                // Da li postoji presek između naših trenutno aktivnih linija i linija sledećeg koraka?
                uint64_t intersection = activeLines & nextStepLines;

                if (intersection != 0) {
                    // Nastavljamo sa postojećom linijom
                    activeLines = intersection;
                } else {
                    // Presedanje! Ispisujemo prethodni segment.
                    printSegment(path, segmentStartIdx, i, activeLines);

                    // Započinjemo novi segment
                    segmentStartIdx = i;
                    activeLines = nextStepLines;
                }
            }

            // Ispisujemo poslednji segment puta do cilja
            printSegment(path, segmentStartIdx, path.size() - 1, activeLines);
        }

        // Pomoćna funkcija za lepši ispis segmenata sa shortName linija
        void printSegment(const std::vector<int>& path, size_t startIdx, size_t endIdx, uint64_t lines) {
            std::cout << "Od stanice " << stations[path[startIdx]].name 
                      << " do stanice " << stations[path[endIdx]].name 
                      << " koristite liniju(e): ";

            bool first = true;
            for (size_t i = 0; i < routes.size() && i < 64; ++i) {
                // Proveravamo da li je setovan bit za i-tu rutu
                if ((lines & (1ULL << i)) != 0) {
                    if (!first) {
                        std::cout << ", ";
                    }
                    std::cout << routes[i].shortName;
                    first = false;
                }
            }
            if (first) {
                std::cout << "Nepoznato";
            }
            std::cout << "\n";
        }

        void printStations() {
            for (size_t i = 0; i < V; ++i) {
                const auto& station = stations[i];
                std::cout << "Index: " << i 
                          << ", ID: " << station.id 
                          << ", Name: " << station.name 
                          << ", Bitmask linija: " << station.lineMask
                          << std::endl 
                          << "----------------------------------------" 
                          << std::endl;
            }
        }

    private:
        int V;
        std::vector<std::vector<std::pair<int, double>>> adjacency_list;
        std::vector<Station> stations;
        std::vector<Route> routes;
        std::unordered_map<std::string, int> stationsToIndex; // Mapa za brzo pronalaženje indeksa stanice po njenom ID-u
        std::unordered_map<std::string, int> routesToIndex;   // Mapa za brzo pronalaženje indeksa rute po njenom ID-u
        std::vector<ParentInfo> parents;
        std::unordered_map<std::string, int> nameToIndex;
};

int main() {

    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    std::string routesFile = "routes.txt";
    std::string stopsFile = "stops.txt";
    std::string edgesFile = "edges.txt";

    Graph g(routesFile, stopsFile, edgesFile);
    std::unordered_map<std::string, int> nameToIndex = g.getNameToIndexMap();

    std::cout << "Da li zelite da ispisete sve stanice? (da/ne): ";
    std::string response;
    std::getline(std::cin, response);
    if (response == "da") {
        g.printStations();
    }
    
    std::cout << "Unesite ime početne stanice: ";
    std::string startName;
    std::getline(std::cin, startName);

    if (nameToIndex.find(startName) == nameToIndex.end()) {
        std::cout << "Greška: Početna stanica \"" << startName << "\" ne postoji!\n";
        return 1;
    }
    int startID = nameToIndex[startName];

    std::cout << "Unesite ime ciljne stanice: ";
    std::string goalName;
    std::getline(std::cin, goalName);

    if (nameToIndex.find(goalName) == nameToIndex.end()) {
        std::cout << "Greška: Ciljna stanica \"" << goalName << "\" ne postoji!\n";
        return 1;
    }
    int goalID = nameToIndex[goalName];

    g.AStar(startID, goalID);
    return 0;
}