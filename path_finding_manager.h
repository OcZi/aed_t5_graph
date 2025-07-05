//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
#define HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H


#include <functional>
#include <iostream>
#include <queue>

#include "window_manager.h"
#include "graph.h"
#include <unordered_map>
#include <unordered_set>


// Este enum sirve para identificar el algoritmo que el usuario desea simular
enum Algorithm
{
    None,
    Dijkstra,
    AStar,
    BFS
};


//* --- PathFindingManager ---
//
// Esta clase sirve para realizar las simulaciones de nuestro grafo.
//
// Variables miembro
//     - path           : Contiene el camino resultante del algoritmo que se desea simular
//     - visited_edges  : Contiene todas las aristas que se visitaron en el algoritmo, notar que 'path'
//                        es un subconjunto de 'visited_edges'.
//     - window_manager : Instancia del manejador de ventana, es utilizado para dibujar cada paso del algoritmo
//     - src            : Nodo incial del que se parte en el algoritmo seleccionado
//     - dest           : Nodo al que se quiere llegar desde 'src'
//*
class PathFindingManager
{
    WindowManager* window_manager;
    std::vector<sfLine> path;
    std::vector<sfLine> visited_edges;

    struct Entry
    {
        Node* node;
        double dist;

        bool operator <(const Entry& other) const
        {
            return dist < other.dist;
        }

        bool operator >(const Entry& other) const
        {
            return dist > other.dist;
        }
    };

    static constexpr double INF = std::numeric_limits<double>::max();

    void bfs(const Graph& graph)
    {
        std::unordered_map<Node*, Node*> parent; // O(1)
        std::priority_queue<Entry, std::vector<Entry>, std::greater<>> queue;
        std::unordered_set<Node*> visited; // O(1)
        int render_counter{}; // O(1)

        auto f = [&](const Node* node) -> double
        {
            double dx = dest->coord.x - node->coord.x; // O(1)
            double dy = dest->coord.y - node->coord.y; // O(1)
            return std::sqrt(dx * dx + dy * dy); // O(1)
        };

        queue.push({src, f(src)}); // O(1)
        visited.insert(src); // O(1)

        while (!queue.empty()) // O(V)
        {
            auto [curr_node, dist] = queue.top(); // O(1)
            queue.pop(); // O(1)

            if (curr_node == dest) break; // O(1)

            for (const auto& edge : curr_node->edges) // Recorre cada edge del nodo, O(grad(curr_node))
            {
                Node* adj = (edge->src == curr_node) ? edge->dest : edge->src; // O(1)
                if (edge->one_way && edge->src != curr_node || visited.count(adj)) continue; // O(1 + 1)

                visited.insert(adj); // O(1)
                parent[adj] = curr_node; // O(1)

                // Draw each edge per iteration
                visited_edges.emplace_back(curr_node->coord, adj->coord, edge->color, edge->thickness); // O(1)
                if (++render_counter % 100 == 0) render(); // O(1)

                queue.push({adj, f(adj)}); // O(1)
            }

        }

        set_final_path(parent); // O(P), con P ≤ V
        // Cada vértice se explora como máximo una sola vez.
        // Cada arista se explora una sola vez gracias al visited set.
        // => O(bfs) = O(V + E + P), para P ≤ V
    }

    /*
    Graph
        std::map<size_t, Node *> nodes;
        std::vector<Edge *> edges;
    */

    void dijkstra(Graph& graph)
    {
        std::unordered_map<Node*, Node*> parent; // O(1)
        std::unordered_map<Node*, double> dist; // O(1)
        int render_counter{}; // O(1)

        for (auto& [id, node] : graph.nodes) // O(V)
        {
            dist[node] = INF;
        }
        dist[src] = 0;

        std::priority_queue<Entry, std::vector<Entry>, std::greater<>> q; // O(1)

        q.push({src, 0}); // O(1)

        // O(V)
        while (!q.empty()) // Se ejecuta 'E', pero es aceptado 'V' veces por el check de mejor distancia.
        {
            auto [curr_node, curr_dist] = q.top(); // O(1)
            q.pop(); // O(log V), aplica heapify

            if (curr_dist > dist[curr_node]) // check de mejor distancia, O(1)
            {
                continue; // no vale
            }

            if (curr_node == dest) // O(1)
            {
                break;
            }

            for (const auto& edge : curr_node->edges) // O(E)
            {
                Node* adj = (edge->src == curr_node) ? edge->dest : edge->src; // O(1)

                if (edge->one_way && edge->src != curr_node) // O(1)
                {
                    continue;
                }

                double new_dist = dist[curr_node] + edge->length; // O(1)

                if (new_dist < dist[adj]) // O(1)
                {
                    dist[adj] = new_dist; // O(1)
                    parent[adj] = curr_node; // O(1)

                    visited_edges.emplace_back( // O(1)
                        curr_node->coord,
                        adj->coord,
                        edge->color,
                        edge->thickness
                    );
                    if (++render_counter % 100 == 0) render(); // O(1)

                    q.push({adj, new_dist}); // O(log V), hasta O(E) inserts
                }
            }
        }

        set_final_path(parent); // O(P), para P ≤ V
        // Dijkstra con cola de prioridad (min-heap):
        // Cada nodo se procesa a lo más una vez válidamente.
        // Cada arista se relaja a lo más una vez.
        // Cola de prioridad maneja entradas con costo O(log V).
        // => Complejidad: O((V + E) * log V)
    }

    void a_star(Graph& graph)
    {
        std::unordered_map<Node*, Node*> parent; // O(1)
        std::unordered_map<Node*, double> dist; // O(1)
        int render_counter{}; // O(1)

        auto h = [&](Node* n) // Función heurística
        {
            double dx = dest->coord.x - n->coord.x; // O(1)
            double dy = dest->coord.y - n->coord.y; // O(1)
            return std::sqrt(dx * dx + dy * dy); // euclidean, O(1)
        };

        for (auto& [id, node] : graph.nodes) // O(V)
        {
            dist[node] = INF;
        }
        dist[src] = 0.0;

        std::priority_queue<Entry, std::vector<Entry>, std::greater<>> q;

        q.push({src, dist[src] + h(src)}); // O(log V)

        while (!q.empty()) // O(V)
        {
            auto [curr_node, node_dist] = q.top(); // O(1)
            q.pop(); // O(log V)

            if (node_dist > dist[curr_node] + h(curr_node)) continue; // O(1)

            if (curr_node == dest) break; // O(1)

            for (const auto& edge : curr_node->edges) // O(grad(curr_node)), en total O(E)
            {
                Node* adj = (edge->src == curr_node) ? edge->dest : edge->src; // O(1)

                double new_dist = dist[curr_node] + edge->length; // O(1)

                if (edge->one_way && edge->src != curr_node) { // O(1)
                    continue;
                }


                if (new_dist < dist[adj])
                {
                    dist[adj] = new_dist; // O(1)
                    parent[adj] = curr_node; // O(1)


                    visited_edges.emplace_back(curr_node->coord, // O(1)
                                               adj->coord,
                                               edge->color,
                                               edge->thickness
                    );

                    if (++render_counter % 100 == 0) render(); // O(1)

                    q.push({adj, new_dist + h(adj)}); // O(log V)
                }
            }
        }

        set_final_path(parent); // O(P), donde P ≤ V
    }

    //* --- render ---
    // En cada iteración de los algoritmos esta función es llamada para dibujar los cambios en el 'window_manager'
    void render()
    {
        sf::sleep(sf::milliseconds(1));
        auto& w = window_manager->get_window();

        // Black screen it to show only the path
        w.clear(sf::Color::Black);
        draw();
        w.display();
    }

    //* --- set_final_path ---
    // Esta función se usa para asignarle un valor a 'this->path' al final de la simulación del algoritmo.
    // 'parent' es un std::unordered_map que recibe un puntero a un vértice y devuelve el vértice anterior a el,
    // formando así el 'path'.
    //
    // ej.
    //     parent(a): b
    //     parent(b): c
    //     parent(c): d
    //     parent(d): NULL
    //
    // Luego, this->path = [Line(a.coord, b.coord), Line(b.coord, c.coord), Line(c.coord, d.coord)]
    //
    // Este path será utilizado para hacer el 'draw()' del 'path' entre 'src' y 'dest'.
    //*
    void set_final_path(std::unordered_map<Node*, Node*>& parent)
    {
        std::vector<sfLine> newPath;
        Node* current = dest;

        while (current && parent.count(current))
        {
            Node* prev = parent[current];
            if (prev == current || prev == nullptr) break;
            newPath.emplace_back(current->coord, prev->coord, sf::Color::Blue, default_thickness);
            current = prev;
        }

        path = newPath;
    }

public:
    Node* src = nullptr;
    Node* dest = nullptr;

    explicit PathFindingManager(WindowManager* window_manager) : window_manager(window_manager)
    {
    }

    void exec(Graph& graph, Algorithm algorithm)
    {
        if (src == nullptr || dest == nullptr)
        {
            return;
        }

        if (algorithm == None)
        {
            return;
        }

        window_manager->set_extra_lines(true);
        visited_edges.clear();
        switch (algorithm)
        {
        case None: // just because C++ warning
        case Dijkstra:
            dijkstra(graph);
            return;
        case AStar:
            a_star(graph);
            return;
        case BFS:
            bfs(graph);
        }
    }

    void reset()
    {
        path.clear();
        visited_edges.clear();

        if (src)
        {
            src->reset();
            src = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
        if (dest)
        {
            dest->reset();
            dest = nullptr;
            // ^^^ Pierde la referencia luego de restaurarlo a sus valores por defecto
        }
    }

    void draw()
    {
        // Dibujar todas las aristas visitadas
        if (window_manager->show_extra_lines())
        {
            for (sfLine& line : visited_edges)
            {
                line.draw(window_manager->get_window(), sf::RenderStates::Default);
            }
        }

        // Dibujar el camino resultante entre 'str' y 'dest'
        for (sfLine& line : path)
        {
            line.draw(window_manager->get_window(), sf::RenderStates::Default);
        }

        // Dibujar el nodo inicial
        if (src != nullptr)
        {
            src->draw(window_manager->get_window());
        }

        // Dibujar el nodo final
        if (dest != nullptr)
        {
            dest->draw(window_manager->get_window());
        }
    }
};


#endif //HOMEWORK_GRAPH_PATH_FINDING_MANAGER_H
