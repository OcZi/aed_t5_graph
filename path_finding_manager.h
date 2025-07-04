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
#include <set>


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

    void bfs(const Graph& graph) // SOLO H
    {
        std::unordered_map<Node*, Node*> parent;

        std::queue<Entry> queue;
        std::set<Node*> visited;
        int render_counter{};

        auto f = [&](Node* n) -> double {
            double dx = dest->coord.x - n->coord.x;
            double dy = dest->coord.y - n->coord.y;
            double euclidean_dist = std::sqrt(dx * dx + dy * dy);
            return euclidean_dist / graph.max_speed_in_graph;  // Usa la velocidad del grafo
        };

        queue.push({src, f(src)});
        visited.insert(src);

        while (!queue.empty())
        {
            auto [curr_node, dist] = queue.front();
            queue.pop();

            if (curr_node == dest)
            {
                std::cout << "BFS Done" << std::endl;
                break;
            }

            for (const auto& edge : curr_node->edges)
            {
                Node* adj = (edge->src == curr_node) ? edge->dest : edge->src;
                if (edge->one_way && edge->src != curr_node || visited.count(adj)) continue;

                visited.insert(adj);
                parent[adj] = curr_node;

                // Draw each edge per iteration
                visited_edges.emplace_back(curr_node->coord, adj->coord, edge->color, edge->thickness);
                if (++render_counter % 100 == 0) render();

                queue.push({adj, f(adj)});
            }

        }

        set_final_path(parent);
    }

    /*
    Graph
        std::map<size_t, Node *> nodes;
        std::vector<Edge *> edges;
    */

    void dijkstra(Graph& graph) // SOLO G
    {
        std::unordered_map<Node*, Node*> parent;
        std::unordered_map<Node*, double> dist;
        int render_counter{};

        for (auto& [id, node] : graph.nodes)
        {
            dist[node] = INF;
        }
        dist[src] = 0;

        std::priority_queue<Entry, std::vector<Entry>, std::greater<>> q;

        q.push({src, 0});

        while (!q.empty())
        {
            auto [curr_node, curr_dist] = q.top();
            q.pop();

            if (curr_dist > dist[curr_node])
            {
                continue; // no vale
            }

            if (curr_node == dest)
            {
                break;
            }

            for (const auto& edge : curr_node->edges)
            {
                Node* adj = (edge->src == curr_node) ? edge->dest : edge->src;

                if (edge->one_way && edge->src != curr_node)
                {
                    continue;
                }

                double new_dist = dist[curr_node] + edge->get_travel_time();

                if (new_dist < dist[adj])
                {
                    dist[adj] = new_dist;
                    parent[adj] = curr_node;

                    visited_edges.emplace_back(
                        curr_node->coord,
                        adj->coord,
                        edge->color,
                        edge->thickness
                    );
                    if (++render_counter % 100 == 0) render();

                    q.push({adj, new_dist});
                }
            }
        }

        set_final_path(parent);
    }

 void a_star(Graph& graph)
    {
        std::unordered_map<Node*, Node*> parent;
        std::unordered_map<Node*, double> dist;
        int render_counter{};

        auto h = [&](Node* n) -> double {
            double dx = dest->coord.x - n->coord.x;
            double dy = dest->coord.y - n->coord.y;
            double euclidean_dist = std::sqrt(dx * dx + dy * dy);
            return euclidean_dist / graph.max_speed_in_graph;
        };

        for (auto& [id, node] : graph.nodes)
        {
            dist[node] = INF;
        }
        dist[src] = 0.0;

        std::priority_queue<Entry, std::vector<Entry>, std::greater<>> q;

        q.push({src, dist[src] + h(src)});

        while (!q.empty())
        {
            auto [curr_node, f_curr] = q.top();
            q.pop();

            if (f_curr > dist[curr_node] + h(curr_node) + 1e-9)
            {
                continue;
            }

            if (curr_node == dest) break;

            for (const auto& edge : curr_node->edges)
            {
                Node* adj = (edge->src == curr_node) ? edge->dest : edge->src;

                if (edge->one_way && edge->src != curr_node) {
                    continue;
                }

                double new_g = dist[curr_node] + edge->get_travel_time();

                if (new_g < dist[adj])
                {
                    dist[adj] = new_g;
                    parent[adj] = curr_node;

                    visited_edges.emplace_back(curr_node->coord,
                                               adj->coord,
                                               edge->color,
                                               edge->thickness
                    );

                    if (++render_counter % 100 == 0) render();

                    q.push({adj, new_g + h(adj)});
                }
            }
        }

        set_final_path(parent);
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
            std::cout << "drawing goal path " << current->coord.x << ", " << current->coord.y << std::endl;
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
