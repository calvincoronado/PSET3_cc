#ifndef M_PI
#define M_PI 3.14159265358979323846
#define RADIUS 5.0

#endif
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <SFML/Network.hpp> 

#include <winsock2.h>
#include <ws2tcpip.h>

#include "imgui.h"
#include "imgui-SFML.h"

#include <vector>
#include <cmath>
#include <random>
#include <iostream>
#include <string>
#include <chrono>
#include <mutex>
#include <queue>
#include <atomic>
#include <thread>
#include <functional>
#include <condition_variable>
#include <utility>
#include <filesystem>
#include <stdexcept>
#include <Windows.h>


#include <ctime>
#include <fstream>
#include <sstream> // for std::stringstream

template<typename T>
const T& clamp(const T& value, const T& min, const T& max) {
    return (value < min) ? min : ((max < value) ? max : value);
}
namespace fs = std::filesystem;

class ThreadPool {
public:
    ThreadPool(size_t numThreads) : stop(false) {
        for (size_t i = 0; i < numThreads; ++i) {
            threads.emplace_back([this] {
                while (true) {
                    std::function<void()> task;

                    {
                        std::unique_lock<std::mutex> lock(queueMutex);
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });
                        if (stop && tasks.empty()) {
                            return;
                        }
                        task = std::move(tasks.front());
                        tasks.pop();
                    }

                    task();
                }
                });
        }
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread& worker : threads) {
            worker.join();
        }
    }

    template <class F, class... Args>
    void enqueue(F&& f, Args&&... args) {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            tasks.emplace([=]() mutable { std::forward<F>(f)(std::forward<Args>(args)...); });
        }
        condition.notify_one();
    }

private:
    std::vector<std::thread> threads;
    std::queue<std::function<void()>> tasks;
    std::mutex queueMutex;
    std::condition_variable condition;
    bool stop;
};

class Particle {
public:
    Particle(float startX, float startY, float speed, float angle)
        : position(startX, startY), velocity(speed* std::cos(angle), speed* std::sin(angle)), isCollided(false) {}

    Particle(const Particle& other)
        : position(other.position), velocity(other.velocity) {}

    Particle& operator=(const Particle& other) {
        if (this != &other) {
            std::lock(mutex, other.mutex);
            std::lock_guard<std::mutex> self_lock(mutex, std::adopt_lock);
            std::lock_guard<std::mutex> other_lock(other.mutex, std::adopt_lock);
            position = other.position;
            velocity = other.velocity;
        }
        return *this;
    }

    void update(float deltaTime, float canvasWidth, float canvasHeight, const std::vector<sf::VertexArray>& walls) {
        std::lock_guard<std::mutex> lock(mutex);

        sf::Vector2f nextPosition = position + velocity * deltaTime;

        if (nextPosition.x < 0 || nextPosition.x > canvasWidth) {
            velocity.x = -velocity.x;
            nextPosition.x = clamp(nextPosition.x, 0.0f, static_cast<float>(canvasWidth));
        }
        if (nextPosition.y < 0 || nextPosition.y > canvasHeight) {
            velocity.y = -velocity.y;
            nextPosition.y = clamp(nextPosition.y, 0.0f, static_cast<float>(canvasHeight));
        }
        if (!isCollided) {
            for (const auto& wall : walls) {
                for (size_t i = 0; i < wall.getVertexCount() - 1; ++i) {
                    sf::Vector2f p1 = wall[i].position;
                    sf::Vector2f p2 = wall[i + 1].position;
                    if (intersects(position, nextPosition, p1, p2)) {
                        isCollided = true;


                        sf::Vector2f normal = getNormal(p1, p2);

                        float dotProduct = velocity.x * normal.x + velocity.y * normal.y;
                        sf::Vector2f reflection = velocity - 2.0f * dotProduct * normal;
                        nextPosition = getCollisionPoint(position, nextPosition, p1, p2);

                        velocity = reflection;
                        break;
                    }
                }
            }
        }
        else {
            isCollided = false;
        }
        position = nextPosition;
    }

    

    sf::Vector2f getPosition() const {
        return position;
    }
    sf::Vector2f getVelocity() const {
        return velocity;
    }

    //no longer in use
    void render(sf::RenderWindow& window) const {
        std::lock_guard<std::mutex> lock(mutex);

        sf::CircleShape shape(5.0f);
        shape.setFillColor(sf::Color::Green);
        shape.setPosition(position);
        window.draw(shape);
    }

private:
    sf::Vector2f position;
    sf::Vector2f velocity;
    mutable std::mutex mutex;
    bool isCollided;

    sf::Vector2f getCollisionPoint(const sf::Vector2f& startPos, const sf::Vector2f& endPos, const sf::Vector2f& wallStart, const sf::Vector2f& wallEnd) {
        sf::Vector2f collisionPoint;

        float x1 = startPos.x, y1 = startPos.y;
        float x2 = endPos.x, y2 = endPos.y;
        float x3 = wallStart.x, y3 = wallStart.y;
        float x4 = wallEnd.x, y4 = wallEnd.y;

        float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        if (denom == 0) {
            return endPos;
        }

        float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
        float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            collisionPoint.x = x1 + t * (x2 - x1);
            collisionPoint.y = y1 + t * (y2 - y1);
        }
        else {
            return endPos;
        }

        return collisionPoint;
    }

    static bool intersects(const sf::Vector2f& p1, const sf::Vector2f& p2, const sf::Vector2f& q1, const sf::Vector2f& q2) {
        float s1_x, s1_y, s2_x, s2_y;
        s1_x = p2.x - p1.x;
        s1_y = p2.y - p1.y;
        s2_x = q2.x - q1.x;
        s2_y = q2.y - q1.y;
        float s, t;
        s = (-s1_y * (p1.x - q1.x) + s1_x * (p1.y - q1.y)) / (-s2_x * s1_y + s1_x * s2_y);
        t = (s2_x * (p1.y - q1.y) - s2_y * (p1.x - q1.x)) / (-s2_x * s1_y + s1_x * s2_y);

        return s >= 0 && s <= 1 && t >= 0 && t <= 1;
    }

    sf::Vector2f getNormal(const sf::Vector2f& p1, const sf::Vector2f& p2) {
        sf::Vector2f direction = p2 - p1;

        sf::Vector2f normal(-direction.y, direction.x); // Rotate direction vector 90 degrees

        float length = std::sqrt(normal.x * normal.x + normal.y * normal.y);
        if (length != 0) {
            normal.x /= length;
            normal.y /= length;
        }

        return normal;
    }
};

void renderWalls(sf::RenderWindow& window,
    const std::vector<sf::VertexArray>& walls,
    std::mutex& mutex,
    float scale) {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto& wall : walls) {
        // Create a transformed copy of the wall vertices with the given scale
        sf::VertexArray scaledWall(wall.getPrimitiveType());
        for (size_t i = 0; i < wall.getVertexCount(); ++i) {
            scaledWall.append(sf::Vertex(sf::Vector2f(wall[i].position.x * scale, wall[i].position.y * scale)));
        }
        window.draw(scaledWall);
    }
}

void renderParticles(const std::vector<Particle>& particles,
    sf::RenderWindow& window,
    std::mutex& mutex,
    float scale) {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto& particle : particles) {
        sf::CircleShape particleShape(5.0f * scale); // Adjust particle size based on scale
        sf::Vector2f particlePosition = particle.getPosition();
        particleShape.setPosition(particlePosition);
        particleShape.setFillColor(sf::Color::Green);
        window.draw(particleShape);
    }
}


float dot(const sf::Vector2f& v1, const sf::Vector2f& v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

float distance(const sf::Vector2f& v1, const sf::Vector2f& v2) {
    return std::sqrt((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y));
}

sf::Vector2f getClosestPointOnSegment(const sf::Vector2f& point, const sf::Vector2f& segmentStart, const sf::Vector2f& segmentEnd) {
    sf::Vector2f segment = segmentEnd - segmentStart;
    float lengthSquared = segment.x * segment.x + segment.y * segment.y; // Compute squared length directly
    if (lengthSquared == 0) {
        return segmentStart;
    }
    float t = std::max(0.0f, std::min(1.0f, dot(point - segmentStart, segment) / lengthSquared));
    return segmentStart + t * segment;
}

bool collidesWithWalls(const sf::Vector2f& position, const std::vector<sf::VertexArray>& walls, float canvasWidth, float canvasHeight) {
    for (const auto& wall : walls) {
        for (size_t i = 0; i < wall.getVertexCount() - 1; ++i) {
            sf::Vector2f p1 = wall[i].position;
            sf::Vector2f p2 = wall[i + 1].position;
            p1.x -= RADIUS;
            p1.y -= RADIUS;
            p2.x -= RADIUS;
            p2.y -= RADIUS;
            sf::Vector2f closestPoint = getClosestPointOnSegment(position, p1, p2);



            if (distance(position, closestPoint) < RADIUS) {
                return true; // Collision detected
            }
        }
    }
    // Check if the position is outside the canvas boundaries
    if (position.x < 0 || position.x >= canvasWidth || position.y < 0 || position.y >= canvasHeight) {
        return true; // Collision detected with canvas boundaries
    }
    return false; // No collision detected
}

void handleInput(sf::CircleShape& ball, float canvasWidth, float canvasHeight, const std::vector<sf::VertexArray>& walls, SOCKET clientSocket) {
    const float speed = 5.0f;

//    sf::Vector2f ballPosition = ball.getPosition();
//    sf::Packet packet;
   

    
    //    std::cout << developerMode << std::endl;
    while (true) {
        //    if (!developerMode) {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W) && ball.getPosition().y >= 0) {
            sf::Vector2f nextPosition = ball.getPosition();

            nextPosition.y -= speed;
            if (!collidesWithWalls(nextPosition, walls, canvasWidth, canvasHeight)) {
                ball.move(0, -speed);
            }

            sf::Vector2f ballPosition = ball.getPosition(); // Get updated ball position

            std::cout << "Sending position: x = " << ballPosition.x << ", y = " << nextPosition.y << std::endl;

            sf::Packet packet;
            packet << ballPosition.x << nextPosition.y;

            // Send the packet to the server (non-blocking)
            const char* data = static_cast<const char*>(packet.getData());
            std::size_t dataSize = packet.getDataSize();

            int bytesSent = send(clientSocket, data, dataSize, 0);
            if (bytesSent == SOCKET_ERROR) {
                if (WSAGetLastError() != WSAEWOULDBLOCK) {
                    std::cerr << "Failed to send position data to the server!" << std::endl;
                }
            }
            else {
                std::cout << "Position data sent to server." << std::endl;
            }

            // Wait for key release before checking for key press again
//            while (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {}


        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A) && ball.getPosition().x >= 0) {
            sf::Vector2f nextPosition = ball.getPosition();
            nextPosition.x -= speed;
            if (!collidesWithWalls(nextPosition, walls, canvasWidth, canvasHeight)) {
                ball.move(-speed, 0);
            }

            sf::Vector2f ballPosition = ball.getPosition(); // Get updated ball position

            std::cout << "Sending position: x = " << nextPosition.x << ", y = " << ballPosition.y << std::endl;

            sf::Packet packet;
            packet << nextPosition.x << ballPosition.y;

            // Send the packet to the server (non-blocking)
            const char* data = static_cast<const char*>(packet.getData());
            std::size_t dataSize = packet.getDataSize();

            int bytesSent = send(clientSocket, data, dataSize, 0);
            if (bytesSent == SOCKET_ERROR) {
                if (WSAGetLastError() != WSAEWOULDBLOCK) {
                    std::cerr << "Failed to send position data to the server!" << std::endl;
                }
            }
            else {
                std::cout << "Position data sent to server." << std::endl;
            }

//            while (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {}

        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S) && ball.getPosition().y + ball.getRadius() + RADIUS < canvasHeight) {
            sf::Vector2f nextPosition = ball.getPosition();
            nextPosition.y += speed;
            if (!collidesWithWalls(nextPosition, walls, canvasWidth, canvasHeight)) {
                ball.move(0, speed);
            }

            sf::Vector2f ballPosition = ball.getPosition(); // Get updated ball position

            std::cout << "Sending position: x = " << ballPosition.x << ", y = " << nextPosition.y << std::endl;

            sf::Packet packet;
            packet << ballPosition.x << nextPosition.y;

            // Send the packet to the server (non-blocking)
            const char* data = static_cast<const char*>(packet.getData());
            std::size_t dataSize = packet.getDataSize();

            int bytesSent = send(clientSocket, data, dataSize, 0);
            if (bytesSent == SOCKET_ERROR) {
                if (WSAGetLastError() != WSAEWOULDBLOCK) {
                    std::cerr << "Failed to send position data to the server!" << std::endl;
                }
            }
            else {
                std::cout << "Position data sent to server." << std::endl;
            }

 //           while (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {}
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::D) &&
            ball.getPosition().x + ball.getRadius() + RADIUS < canvasWidth) {
            sf::Vector2f nextPosition = ball.getPosition();
            nextPosition.x += speed;
            if (!collidesWithWalls(nextPosition, walls, canvasWidth, canvasHeight)) {
                ball.move(speed, 0);
            }

            sf::Vector2f ballPosition = ball.getPosition(); // Get updated ball position

            std::cout << "Sending position: x = " << nextPosition.x << ", y = " << ballPosition.y << std::endl;

            sf::Packet packet;
            packet << nextPosition.x << ballPosition.y;

            // Send the packet to the server (non-blocking)
            const char* data = static_cast<const char*>(packet.getData());
            std::size_t dataSize = packet.getDataSize();

            int bytesSent = send(clientSocket, data, dataSize, 0);
            if (bytesSent == SOCKET_ERROR) {
                if (WSAGetLastError() != WSAEWOULDBLOCK) {
                    std::cerr << "Failed to send position data to the server!" << std::endl;
                }
            }
            else {
                std::cout << "Position data sent to server." << std::endl;
            }

//            while (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {}
        }

    /*    // Create a packet to send ball position
        sf::Vector2f ballPosition = ball.getPosition();
        sf::Packet packet;
        packet << ballPosition.x << ballPosition.y;

        // Send the packet to the server (non-blocking)
        const char* data = static_cast<const char*>(packet.getData());
        std::size_t dataSize = packet.getDataSize();

        int bytesSent = send(clientSocket, data, dataSize, 0);
        if (bytesSent == SOCKET_ERROR) {
            if (WSAGetLastError() != WSAEWOULDBLOCK) {
                std::cerr << "Failed to send position data to the server!" << std::endl;
            }
        }
        else {
            std::cout << "Position data sent to server." << std::endl;
        } */


        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void receiveDataFromServer(SOCKET clientSocket, std::vector<Particle>& particles) {
    while (true) {
        sf::Vector2f position;
        float receivedSpeed;
        float receivedAngle;

        // Receive particle position
        if (recv(clientSocket, reinterpret_cast<char*>(&position), sizeof(position), 0) == SOCKET_ERROR) {
            std::cerr << "Error receiving particle position." << std::endl;
            break;
        }

        // Receive particle speed
        if (recv(clientSocket, reinterpret_cast<char*>(&receivedSpeed), sizeof(receivedSpeed), 0) == SOCKET_ERROR) {
            std::cerr << "Error receiving particle speed." << std::endl;
            break;
        }

        // Receive particle angle
        if (recv(clientSocket, reinterpret_cast<char*>(&receivedAngle), sizeof(receivedAngle), 0) == SOCKET_ERROR) {
            std::cerr << "Error receiving particle angle." << std::endl;
            break;
        }

    //    std::lock_guard<std::mutex> lock(mutex);

        // Update the received data
    //    speed = receivedSpeed;
    //    angle = receivedAngle;

        // Handle received data here
        // For example, you can add the received particle to the particles vector
        particles.emplace_back(position.x, position.y, receivedSpeed, receivedAngle);
    }
} 

std::string signalToClear(SOCKET serverSocket) {
    char buffer[4096];
    memset(buffer, 0, sizeof(buffer));
    recv(serverSocket, buffer, sizeof(buffer), 0);
    return std::string(buffer);
}


int main() {

    // Initialize WSA variables
    WSADATA wsaData;
    int wsaerr;
    WORD wVersionRequested = MAKEWORD(2, 2);
    wsaerr = WSAStartup(wVersionRequested, &wsaData);
    if (wsaerr != 0) {
        std::cout << "The Winsock dll not found" << std::endl;
        return 0;
    }
    else {
        std::cout << "The Winsock dll found" << std::endl;
        std::cout << "The status: " << wsaData.szSystemStatus << std::endl;
    }

    // Create socket
    SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (clientSocket == INVALID_SOCKET) {
        std::cout << "Error at socket(): " << WSAGetLastError() << std::endl;
        WSACleanup();
        return 0;
    }
    else {
        std::cout << "Socket is on" << std::endl;
    }

    // Connect to server
    sockaddr_in service;
    service.sin_family = AF_INET;
    service.sin_addr.s_addr = inet_addr("192.168.68.109"); // Change to server IP address
    service.sin_port = htons(55555); // Change to server port
    if (connect(clientSocket, (SOCKADDR*)&service, sizeof(service)) == SOCKET_ERROR) {
        std::cout << "Failed to connect." << std::endl;
        closesocket(clientSocket);
        WSACleanup();
        return 0;
    }
    else {
        std::cout << "Connected to server." << std::endl;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    sf::RenderWindow window(sf::VideoMode(1280 + 10, 720 + 10), "Particle Bouncing Application"); // adjusting size for aesthetic purposes, canvas walls are still 1280 x 720
    window.setFramerateLimit(60);

    sf::Clock clock;
    sf::Clock deltaClock;
    sf::Clock fpsClock;
    int frameCount = 0;
    float fps = 0;
    auto lastFpsTime = std::chrono::steady_clock::now();

    ImGui::SFML::Init(window);

    std::vector<Particle> particles;
    float canvasWidth = 1280.0f;
    float canvasHeight = 720.0f;
    float speed = 100.0f;
    float startAngle = 0.0f;
    float endAngle = 180.0f;

    float angle = 45.0f * M_PI / 180.0f; // Convert angle to radians
    int numParticles = 1;
    std::vector<sf::VertexArray> walls;

    bool isDrawingLine = false;
    sf::Vector2f lineStart(100.0f, 360.0f); // Default line start point
    sf::Vector2f lineEnd(1180.0f, 360.0f);  // Default line end point

    sf::Vector2f spawnPoint(640.0f, 360.0f); // Default spawn point

    sf::CircleShape ball(RADIUS);
    bool developerMode = true; // Default to developer mode
    ball.setFillColor(sf::Color::Red);
    ball.setPosition(640, 360); // Initial position

    std::thread inputThread(&handleInput,
        std::ref(ball),
        canvasWidth,
        canvasHeight,
        std::ref(walls), clientSocket);

    // Create a packet to send ball position
    sf::Vector2f ballPosition = ball.getPosition();
    sf::Packet packet;
    packet << ballPosition.x << ballPosition.y;

    // Send the packet to the server (non-blocking)
    const char* data = static_cast<const char*>(packet.getData());
    std::size_t dataSize = packet.getDataSize();

    int bytesSent = send(clientSocket, data, dataSize, 0);
    if (bytesSent == SOCKET_ERROR) {
        if (WSAGetLastError() != WSAEWOULDBLOCK) {
            std::cerr << "Failed to send position data to the server!" << std::endl;
        }
    }
    else {
        std::cout << "Position data sent to server." << std::endl;
    }



    unsigned int numThreads = std::thread::hardware_concurrency();
    ThreadPool threadPool(numThreads);

    // Mutex for synchronization
    std::mutex mutex;

    std::thread receiveThread(receiveDataFromServer, clientSocket, std::ref(particles));
 //   std::thread receiveThread(receiveParticles, std::ref(particles), clientSocket, std::ref(mutex));

    

    while (window.isOpen()) {
        sf::Event event;
        
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(event);
            
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            /*    else if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left && developerMode) {
                    if (!ImGui::GetIO().WantCaptureMouse) {
                        if (!isDrawingLine) {
                            isDrawingLine = true;
                            lineStart = window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
                        }
                        else {
                            isDrawingLine = false;
                            lineEnd = window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
                            walls.emplace_back(sf::LinesStrip, 2);
                            walls.back()[0].position = lineStart;
                            walls.back()[1].position = lineEnd;
                        }
                    }
                } */
        }
        float deltaTime = clock.restart().asSeconds();
        ImGui::SFML::Update(window, deltaClock.restart());


        window.clear(sf::Color::Black);

      


        
        // Explorer mode UI
        ImGui::Begin("Explorer Mode");
        ImGui::Separator();

        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastFpsTime).count() / 1000.0;
        if (elapsedTime >= 0.5) {
            double valid_fps = frameCount / elapsedTime;
            fps = valid_fps;
            frameCount = 0;
            lastFpsTime = currentTime;
        }
        ImGui::Text("FPS: %.1f", fps);
        
        /*    if (ImGui::Checkbox("Developer Mode", &developerMode)) {
                //std::cout << developerMode << std::endl;
                if (developerMode) {
                    window.setView(window.getDefaultView());
                }
            } */


        ImGui::End();

        threadPool.enqueue([&particles, deltaTime, canvasWidth, canvasHeight, &walls]() {
            for (auto& particle : particles) {
                particle.update(deltaTime, canvasWidth, canvasHeight, walls);
            }
            });


        
        float scale = 5.0f;
        float zoomedInLeft = ball.getPosition().x - 16 * scale;
        float zoomedInRight = ball.getPosition().x + 16 * scale;
        float zoomedInTop = ball.getPosition().y - 9 * scale;
        float zoomedInBottom = ball.getPosition().y + 9 * scale;



        sf::View zoomedInView(sf::FloatRect(zoomedInLeft,
            zoomedInTop,
            zoomedInRight - zoomedInLeft,
            zoomedInBottom - zoomedInTop));
        window.setView(zoomedInView);

        renderWalls(window, walls, mutex, 1.0f);
        renderParticles(particles, window, mutex, 1.0f);
        
        window.draw(ball);
        //        }



        ImGui::SFML::Render(window);

        window.display();

        frameCount++; 
        /*
        * std::string message = signalToClear(clientSocket);
        if (message == "Clear") {
            particles.clear();

        }
        */
    }

    ImGui::SFML::Shutdown();
    inputThread.join();


    // Cleanup and close socket
    closesocket(clientSocket);
    WSACleanup();

    return 0;
}