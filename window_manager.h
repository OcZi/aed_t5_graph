//
// Created by juan-diego on 3/29/24.
//

#ifndef HOMEWORK_GRAPH_WINDOW_MANAGER_H
#define HOMEWORK_GRAPH_WINDOW_MANAGER_H

#include <SFML/Graphics.hpp>

//*
// ---- Window Manager ----
// Esta clase sirve como wrapper de nuestra instancia de sf::RenderWindow
// para realizar las manipulaciones de la instancia de manera segura.
//*
class WindowManager {
    sf::RenderWindow window;
    bool draw_extra_lines = false;
public:
    explicit WindowManager(int window_width = 600, int window_height = 800) :
            window(sf::VideoMode(window_width, window_height), "Lima City Graph") {
    }

    bool show_extra_lines()
    {
        return draw_extra_lines;
    }

    bool switch_extra_lines()
    {
        draw_extra_lines = !draw_extra_lines;
        return draw_extra_lines;
    }

    void set_extra_lines(bool extra_lines)
    {
        draw_extra_lines = extra_lines;
    }

    bool is_open() {
        return window.isOpen();
    }

    void close() {
        window.close();
    }

    bool poll_event(sf::Event &event) {
        return window.pollEvent(event);
    }

    void clear(sf::Color color = sf::Color::Black) {
        window.clear(color);
    }

    void display() {
        window.display();
    }

    sf::RenderWindow &get_window() {
        return window;
    }
};


#endif //HOMEWORK_GRAPH_WINDOW_MANAGER_H
