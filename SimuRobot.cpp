/******************************************************************************
                              Online C++ Compiler.
               Code, Compile, Run and Debug C++ program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/
#include <bits/stdc++.h>
using namespace std;
using namespace chrono;

/* ================== PARÁMETROS GENERALES ================== */
using Scalar = double;
const Scalar STEP = 0.1;            // desplazamiento elemental ΔH
const Scalar EPSILON = 0.1;         // tolerancia para detección de la meta
const int MAX_PALPADAS = 2000;      // límite de palpadas
const double NOISE_STD = 0.0;       // ruido del sensor (0 = simulación estable)
const int FILTER_N = 3;            // filtro promedio móvil

// Límites reales de exploración (definidos por el problema)
const Scalar MIN_POS = 8.5;
const Scalar MAX_POS = 14.5;

/* ================== SIMULADOR Y ROBOT ================== */

// Simula el entorno físico: posición real A y lectura ruidosa
struct Simulator {
    Scalar truePos;
    double noiseStd;
    std::mt19937 rng;
    Simulator(Scalar truePos_, double noiseStd_ = NOISE_STD)
        : truePos(truePos_), noiseStd(noiseStd_), rng((uint32_t)chrono::high_resolution_clock::now().time_since_epoch().count()) {}
    Scalar sensorReadingAt(Scalar pos) {
        std::normal_distribution<double> gauss(0.0, noiseStd); //Acá seteo que tenga en cuenta sólo 1 mm de presición????
        return fabs(pos - truePos) + gauss(rng);
    }
};

// Representa al robot que se desplaza y palpa
struct RobotInterface {
    Scalar currentPos;
    Simulator* sim;
    deque<Scalar> lastReadings;
    int palpadas = 0;
    string name;

    RobotInterface(Simulator* s = nullptr, string n = "Robot")
        : currentPos(0.0), sim(s), name(n) {}

    void moveTo(Scalar pos) {
        currentPos = pos;
    }

    // Palpada con filtrado (esta función simula la palpada "oficial" y registra)
    Scalar palpate() {
        Scalar raw = sim ? sim->sensorReadingAt(currentPos) : 1e6;
        lastReadings.push_back(raw);
        if ((int)lastReadings.size() > FILTER_N) lastReadings.pop_front();
        Scalar sum = 0;
        for (auto& v : lastReadings) sum += v;
        Scalar filtered = sum / (Scalar)lastReadings.size();
        palpadas++;
        cout << "[" << name << "] Palpada " << palpadas
             << " | Pos=" << currentPos
             << " | Lectura=" << filtered
             << " | Meta=" << (filtered <= EPSILON ? "SI" : "NO") << "\n";
        return filtered;
    }

    // Lectura cruda adicional (no modifica lastReadings ni palpadas).
    // Sirve para confirmar inmediatamente la condición de meta.
    Scalar senseRawNoRecord() {
        return sim ? sim->sensorReadingAt(currentPos) : 1e6;
    }

    bool isGoal(Scalar reading) { return reading <= EPSILON; }
};

inline Scalar round_to_step(Scalar pos) {
    return round(pos / STEP) * STEP;
}

/* ================== BÚSQUEDA EXHAUSTIVA - BFS ================== */
pair<bool, Scalar> exhaustive_bfs(RobotInterface& robot, Scalar B) {
    Scalar start = round_to_step(B);
    robot.moveTo(start);
    Scalar r0 = robot.palpate();
    if (robot.isGoal(r0)) return { true, start };
    // confirm raw just in case filter masked it
    Scalar raw0 = robot.senseRawNoRecord();
    if (raw0 <= EPSILON) {
        cout << "["<<robot.name<<"] Confirmación RAW en start = " << raw0 << " -> META\n";
        return { true, start };
    }

    unordered_set<long long> visited;
    auto key = [&](Scalar p)->long long { return llround(p / STEP); };
    queue<Scalar> q;
    visited.insert(key(start));
    q.push(start);

    while (!q.empty() && robot.palpadas < MAX_PALPADAS) {
        Scalar pos = q.front(); q.pop();
        for (int dir : {1, -1}) {
            Scalar np = pos + dir * STEP;
            if (np < MIN_POS || np > MAX_POS) continue; // límite real
            long long k = key(np);
            if (visited.count(k)) continue;
            visited.insert(k);
            robot.moveTo(np);
            Scalar r = robot.palpate();
            if (robot.isGoal(r)) return { true, np };
            // confirm raw (no registra) para evitar falsos negativos por el filtro
            Scalar raw = robot.senseRawNoRecord();
            if (raw <= EPSILON) {
                cout << "["<<robot.name<<"] Confirmación RAW en " << np << " = " << raw << " -> META\n";
                return { true, np };
            }
            q.push(np);
        }
    }
    return { false, NAN };
}

/* ================== BÚSQUEDA EXHAUSTIVA - RADIAL ================== */
pair<bool, Scalar> exhaustive_radial(RobotInterface& robot, Scalar B) {
    Scalar Bq = round_to_step(B);
    robot.moveTo(Bq);
    Scalar r0 = robot.palpate();
    if (robot.isGoal(r0)) return { true, Bq };
    Scalar raw0 = robot.senseRawNoRecord();
    if (raw0 <= EPSILON) {
        cout << "["<<robot.name<<"] Confirmación RAW en start = " << raw0 << " -> META\n";
        return { true, Bq };
    }

    for (int k = 1; robot.palpadas < MAX_PALPADAS; k++) {
        bool exploredSomething = false;

        // Derecha
        Scalar pr = Bq + k * STEP;
        if (pr <= MAX_POS) {
            exploredSomething = true;
            robot.moveTo(pr);
            Scalar rf = robot.palpate();
            if (robot.isGoal(rf)) return { true, pr };
            Scalar raw = robot.senseRawNoRecord();
            if (raw <= EPSILON) {
                cout << "["<<robot.name<<"] Confirmación RAW en " << pr << " = " << raw << " -> META\n";
                return { true, pr };
            }
        }

        // Izquierda
        Scalar pl = Bq - k * STEP;
        if (pl >= MIN_POS) {
            exploredSomething = true;
            robot.moveTo(pl);
            Scalar rf = robot.palpate();
            if (robot.isGoal(rf)) return { true, pl };
            Scalar raw = robot.senseRawNoRecord();
            if (raw <= EPSILON) {
                cout << "["<<robot.name<<"] Confirmación RAW en " << pl << " = " << raw << " -> META\n";
                return { true, pl };
            }
        }

        if (!exploredSomething) break;
    }
    return { false, NAN };
}

/* ================== BÚSQUEDA HEURÍSTICA - HILL CLIMBING ================== */
pair<bool, Scalar> hill_climbing(RobotInterface& robot, Scalar B) {
    Scalar start = round_to_step(B);
    robot.moveTo(start);
    Scalar rStart = robot.palpate();
    if (robot.isGoal(rStart)) return { true, start };
    Scalar rawStart = robot.senseRawNoRecord();
    if (rawStart <= EPSILON) { cout<<"["<<robot.name<<"] Confirmación RAW start = "<<rawStart<<" -> META\n"; return {true,start}; }

    // Intentar derecha
    Scalar pos = start;
    Scalar best = rStart;
    while (pos + STEP <= MAX_POS && robot.palpadas < MAX_PALPADAS) {
        Scalar next = pos + STEP;
        robot.moveTo(next);
        Scalar r = robot.palpate();
        if (robot.isGoal(r)) return { true, next };
        Scalar raw = robot.senseRawNoRecord();
        if (raw <= EPSILON) { cout<<"["<<robot.name<<"] Confirmación RAW en "<<next<<" = "<<raw<<" -> META\n"; return {true,next}; }
        if (r < best) { pos = next; best = r; } else break;
    }

    // Intentar izquierda
    pos = start;
    best = rStart;
    while (pos - STEP >= MIN_POS && robot.palpadas < MAX_PALPADAS) {
        Scalar next = pos - STEP;
        robot.moveTo(next);
        Scalar r = robot.palpate();
        if (robot.isGoal(r)) return { true, next };
        Scalar raw = robot.senseRawNoRecord();
        if (raw <= EPSILON) { cout<<"["<<robot.name<<"] Confirmación RAW en "<<next<<" = "<<raw<<" -> META\n"; return {true,next}; }
        if (r < best) { pos = next; best = r; } else break;
    }

    return { false, NAN };
}

/* ================== BÚSQUEDA HEURÍSTICA - A* ================== */
pair<bool, Scalar> a_star_search(RobotInterface& robot, Scalar B) {
    struct Node { Scalar f, g, pos; bool operator<(const Node& o) const { return f > o.f; } };
    auto key = [&](Scalar p)->long long { return llround(p / STEP); };

    Scalar start = round_to_step(B);
    robot.moveTo(start);
    Scalar h0 = robot.palpate();
    if (robot.isGoal(h0)) return { true, start };
    Scalar raw0 = robot.senseRawNoRecord();
    if (raw0 <= EPSILON) { cout<<"["<<robot.name<<"] Confirmación RAW start = "<<raw0<<" -> META\n"; return {true,start}; }

    unordered_map<long long,double> best_g;
    priority_queue<Node> open;
    best_g[key(start)] = 0.0;
    open.push({h0, 0.0, start});

    while (!open.empty() && robot.palpadas < MAX_PALPADAS) {
        Node cur = open.top(); open.pop();
        for (int dir : {1, -1}) {
            Scalar np = cur.pos + dir * STEP;
            if (np < MIN_POS || np > MAX_POS) continue;
            double ng = cur.g + STEP;
            if (best_g.count(key(np)) && ng >= best_g[key(np)]) continue;

            robot.moveTo(np);
            Scalar h = robot.palpate();
            if (robot.isGoal(h)) return { true, np };
            Scalar raw = robot.senseRawNoRecord();
            if (raw <= EPSILON) { cout<<"["<<robot.name<<"] Confirmación RAW en "<<np<<" = "<<raw<<" -> META\n"; return {true,np}; }
            best_g[key(np)] = ng;
            open.push({ng + h, ng, np});
        }
    }
    return { false, NAN };
}

/* ================== MAIN ================== */
int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    cout << fixed << setprecision(1); // redondeo

    Scalar B = 10.0;  // posición teórica inicial
    mt19937 rng((uint32_t)chrono::high_resolution_clock::now().time_since_epoch().count());
    uniform_real_distribution<double> distA(MIN_POS, MAX_POS);
    Scalar trueA = distA(rng);  // posición real de montaje entre 8.5 y 14.5

    cout << "Simulación: B=" << B << " | Posición real A=" << trueA << "\n\n";

    vector<pair<string,function<pair<bool,Scalar>(RobotInterface&,Scalar)>>> metodos = {
        {"Exhaustiva BFS", exhaustive_bfs},
        {"Exhaustiva Radial", exhaustive_radial},
        {"Heurística Hill-Climbing", hill_climbing},
        {"Heurística A*", a_star_search}
    };

    vector<pair<string,bool>> resultados;
    vector<Scalar> posiciones;
    vector<int> palpadas;

    for (auto& m : metodos) {
        Simulator sim(trueA, NOISE_STD);
        RobotInterface robot(&sim, m.first);

        auto t1 = high_resolution_clock::now();
        auto [encontrado, pos] = m.second(robot, B);
        auto t2 = high_resolution_clock::now();
        long long ms = duration_cast<milliseconds>(t2 - t1).count();

        resultados.push_back({m.first, encontrado});
        posiciones.push_back(pos);
        palpadas.push_back(robot.palpadas);

        cout << "\n--- " << m.first << " ---\n";
        if (encontrado)
            cout << "Meta encontrada en " << pos << " (error = " << fabs(pos - trueA) << ")\n";
        else
            cout << "No se encontró dentro del rango.\n";
        cout << "Palpadas: " << robot.palpadas << " | Tiempo: " << ms << " ms\n\n";
    }

    cout << "================= RESUMEN =================\n";
    cout << "Simulación: Punto de partida B=" << B << ", Posición real A=" << trueA << "\n";
    cout << "===========================================\n";
    for (size_t i = 0; i < resultados.size(); i++) {
        cout << resultados[i].first << " → "
             << (resultados[i].second ? "ÉXITO" : "FALLO")
             << " | Pos=" << posiciones[i]
             << " | Palpadas=" << palpadas[i] << "\n";
    }
    cout << "===========================================\n";

    return 0;
}
