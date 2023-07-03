// Path_planning_MQTT.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include<cstdlib>
#include<cstring>
#include<mqtt/async_client.h>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

//include for main algorithm
#include <iostream>
#include <cmath>
#include <vector>
#include<limits>
#include <utility>
#include <iomanip>


const std::string SERVER_ADDRESS = "tcp://42.96.40.234:1883";
const std::string ClIENT_ID = "Path_Planning";
const std::string TOPIC_2 = "algorithm_return";
const std::string TOPIC= "user_set_up";
const int QOS = 1;

const int TIMEOUT = 10000;

struct Node
{

public:
    int x, y;
    float G_cost;
    float H_cost;
    float J_cost;
    Node* parent;
    bool check_childNode = 0;
    bool check_visited = 0;

    Node() {}
};

float update_G_cost(Node& A, Node& B)
{
    int x = A.x;
    int x1 = B.x;
    int y = A.y;
    int y1 = B.y;
    float G = A.G_cost + sqrt(pow((x - x1), 2) + pow((y - y1), 2));
    return G;
}

float calculate_H_cost(Node& A, Node& Final) {
    int hx = A.x;
    int hy = A.y;
    int x1 = Final.x;
    int y1 = Final.y;
    float H;
    H = sqrt(pow((hx - x1), 2) + pow((hy - y1), 2));
    return H;
}

float calculate_J_cost(Node& A) {
    float J = A.H_cost + A.G_cost;
    return J;
}

class Map
{
public:
    int Map_x;
    int Map_y;
    Node* point;
    Node* StartNode;
    Node* FinalNode;
    std::vector<Node*> Obs;
    Map(int x, int y) : Map_x(x), Map_y(y) {
        point = new Node[Map_x * Map_y];
        for (int iy = 0; iy < Map_y; iy++) {
            for (int ix = 0; ix < Map_x; ix++) {
                point[iy * Map_x + ix].x = ix;
                point[iy * Map_x + ix].y = iy;
            }
        }
        for (int i = 0; i < Map_x * Map_y; i++) {
            point[i].G_cost = std::numeric_limits<float>::infinity();
        }
    }
};

std::vector<std::pair<int, int>> A_star(Map World_Map) {
    Node* CurrentNode;
    World_Map.StartNode->G_cost = 0;
    CurrentNode = World_Map.StartNode;
    std::vector<Node*> Visited; // queue to store all nodes have been visited
    Visited.push_back(World_Map.StartNode);
    World_Map.StartNode->check_visited = 1;
    std::vector<Node*> Q; // queue to store all nodes have been taken into consideration
    while (CurrentNode != World_Map.FinalNode)
    {
        std::vector<Node*> ChildNodes;
        if (CurrentNode->x - 1 >= 0 && CurrentNode->y + 1 >= 0 && CurrentNode->x - 1 < World_Map.Map_x && CurrentNode->y + 1 < World_Map.Map_y) {
            Node* ChildNode_1 = &World_Map.point[(CurrentNode->y + 1) * World_Map.Map_x + CurrentNode->x - 1];
            ChildNodes.push_back(ChildNode_1);
        }
        if (CurrentNode->x >= 0 && CurrentNode->y + 1 >= 0 && CurrentNode->x < World_Map.Map_x && CurrentNode->y + 1 < World_Map.Map_y) {
            Node* ChildNode_2 = &World_Map.point[(CurrentNode->y + 1) * World_Map.Map_x + CurrentNode->x];
            ChildNodes.push_back(ChildNode_2);
        }
        if (CurrentNode->x + 1 >= 0 && CurrentNode->y + 1 >= 0 && CurrentNode->x + 1 < World_Map.Map_x && CurrentNode->y + 1 < World_Map.Map_y) {
            Node* ChildNode_3 = &World_Map.point[(CurrentNode->y + 1) * World_Map.Map_x + CurrentNode->x + 1];
            ChildNodes.push_back(ChildNode_3);
        }
        if (CurrentNode->x + 1 >= 0 && CurrentNode->y >= 0 && CurrentNode->x + 1 < World_Map.Map_x && CurrentNode->y < World_Map.Map_y) {
            Node* ChildNode_4 = &World_Map.point[(CurrentNode->y) * World_Map.Map_x + CurrentNode->x + 1];
            ChildNodes.push_back(ChildNode_4);
        }
        if (CurrentNode->x + 1 >= 0 && CurrentNode->y - 1 >= 0 && CurrentNode->x + 1 < World_Map.Map_x && CurrentNode->y - 1 < World_Map.Map_y) {
            Node* ChildNode_5 = &World_Map.point[(CurrentNode->y - 1) * World_Map.Map_x + CurrentNode->x + 1];
            ChildNodes.push_back(ChildNode_5);
        }
        if (CurrentNode->x >= 0 && CurrentNode->y - 1 >= 0 && CurrentNode->x < World_Map.Map_x && CurrentNode->y - 1 < World_Map.Map_y) {
            Node* ChildNode_6 = &World_Map.point[(CurrentNode->y - 1) * World_Map.Map_x + CurrentNode->x];
            ChildNodes.push_back(ChildNode_6);
        }
        if (CurrentNode->x - 1 >= 0 && CurrentNode->y - 1 >= 0 && CurrentNode->x - 1 < World_Map.Map_x && CurrentNode->y - 1 < World_Map.Map_y) {
            Node* ChildNode_7 = &World_Map.point[(CurrentNode->y - 1) * World_Map.Map_x + CurrentNode->x - 1];
            ChildNodes.push_back(ChildNode_7);
        }
        if (CurrentNode->x - 1 >= 0 && CurrentNode->y >= 0 && CurrentNode->x - 1 < World_Map.Map_x && CurrentNode->y < World_Map.Map_y) {
            Node* ChildNode_8 = &World_Map.point[(CurrentNode->y) * World_Map.Map_x + CurrentNode->x - 1];
            ChildNodes.push_back(ChildNode_8);
        }
        // update parent node and G_cost of ChildNode
        for (int i = 0; i < ChildNodes.size(); i++) {
            float Temp_G = update_G_cost(*CurrentNode, *ChildNodes[i]);
            if (Temp_G < ChildNodes[i]->G_cost) {
                ChildNodes[i]->G_cost = Temp_G;
                ChildNodes[i]->parent = CurrentNode;
            }
        }
        // Store childnodes into Q queue
        for (int i = 0; i < Q.size(); i++) {
            for (int j = 0; j < ChildNodes.size(); j++) {
                if (ChildNodes[j] == Q[i]) {
                    ChildNodes[j]->check_childNode = 1;
                    break;
                }
            }
        }
        for (int i = 0; i < ChildNodes.size(); i++) {
            if (ChildNodes[i]->check_childNode != 1 && ChildNodes[i]->check_visited != 1) {
                Q.push_back(ChildNodes[i]);
            }
        }
        // Find the node has the lowest J cost in Q queue and assign it as current node
        Node* Q_min;
        Q_min = Q[0];
        for (int i = 0; i < Q.size(); i++) {
            Q[i]->H_cost = calculate_H_cost(*Q[i], *World_Map.FinalNode);
            Q[i]->J_cost = calculate_J_cost(*Q[i]);
        }
        for (int i = 0; i < Q.size(); i++) {
            for (int j = 0; j < World_Map.Obs.size(); j++) {
                if (Q[i] == World_Map.Obs[j]) {
                    Q[i]->J_cost = std::numeric_limits<float>::infinity();
                    break;
                }
            }
        }
        for (int i = 0; i < Q.size(); i++) {
            if (Q[i]->check_visited != 1 && Q[i]->J_cost < Q_min->J_cost) {
                Q_min = Q[i];
            }
        }
        CurrentNode = Q_min;
        CurrentNode->check_visited = 1;
        for (int i = 0; i < Q.size(); i++) {
            if (Q[i]->check_visited == 1) {
                Q.erase(Q.begin() + i);
            }
        }
        Visited.push_back(CurrentNode);
    }

    std::vector<Node*> Path;
    Path.push_back(World_Map.FinalNode);
    Node* trackback;
    trackback = Visited[Visited.size() - 1];

    while (trackback->parent != World_Map.StartNode) {
        trackback = trackback->parent;
        Path.insert(Path.begin(), trackback);
    }
    std::vector<std::pair<int, int>> result;
    for (int i = 0; i < Path.size(); i++) {
        result.push_back(std::make_pair(Path[i]->x, Path[i]->y));
    }

    return result;
}

std::pair<int, int> realWorld_to_myWorld(double long_big, double long_small, double lat_big, double lat_small, double res) {

    double temp_x = (long_big - long_small) / res + 1;
    int myWorld_x = static_cast<int>(temp_x);
    double temp_y = (lat_big - lat_small) / res + 1 ;
    int myWorld_y = static_cast<int>(temp_y);
    return std::make_pair(myWorld_x, myWorld_y);
}

std::pair<double, double> myWorld_to_realWorld(int myWorld_x, int myWorld_y, double res, double lat_small, double long_small) {
    double temp_x = static_cast<double>(myWorld_x);
    double realWorld_x = (temp_x-1) * res + long_small;
    double temp_y = static_cast<double>(myWorld_y);
    double realWorld_y = (temp_y-1) * res + lat_small;
    return std::make_pair(realWorld_x, realWorld_y);
}

class callback : public virtual mqtt::callback {
private:
    mqtt::async_client& client_;
public:
    explicit callback(mqtt::async_client& client) : client_(client) {}

    void connection_lost(const std::string& cause) override {
        std::cout << "\nConnection lost" << std::endl;
        if (!cause.empty())
            std::cout << "\tCause: " << cause << std::endl;
    }

    void delivery_complete(mqtt::delivery_token_ptr token) override {
        std::cout << "\nDelivery complete" << std::endl;
    }

    void message_arrived(mqtt::const_message_ptr msg) override {
        std::vector<std::pair<int, int>> result;
        std::string payload = msg->to_string();
        std::cout << "Received message: " << payload << std::endl;

        // Parse the JSON payload
        rapidjson::Document document;
        document.Parse(payload.c_str());
        
       
        rapidjson::Value& s = document["start"];
        rapidjson::Value& f = document["final"];
               
        rapidjson::Value& sx = s[0];
        rapidjson::Value& sy = s[1];
        rapidjson::Value& fx = f[0];
        rapidjson::Value& fy = f[1];
                   
        double start_x = sx.GetDouble();
        double start_y = sy.GetDouble();
        double final_x = fx.GetDouble();
        double final_y = fy.GetDouble();

        double min_y = 105.74642;
        double min_x = 20.96106;
        double min_x_obs = 20.96122;
        double min_y_obs = 105.74646;
        double max_x_obs = 20.96136;
        double max_y_obs = 105.74661;
        std::pair<int, int> start_point = realWorld_to_myWorld(start_x, min_x, start_y, min_y, 0.00001);
        std::pair<int, int> final_point = realWorld_to_myWorld(final_x, min_x, final_y, min_y, 0.00001);
        std::pair<int, int> obs_min = realWorld_to_myWorld(min_x_obs, min_x, min_y_obs, min_y, 0.00001);
        std::pair<int, int> obs_max = realWorld_to_myWorld(max_x_obs, min_x, max_y_obs, min_y, 0.00001);
        std::cout << start_point.first << " " << start_point.second << std::endl;
        std::cout << final_point.first << " " << final_point.second << std::endl;
        std::cout << obs_min.first << " " << obs_min.second << std::endl;
        std::cout << obs_max.first << " " << obs_max.second << std::endl;
     // main algorithm
        Map mymap(40, 49);
        for (int i = obs_min.second; i < obs_max.second + 1; i++) {
            for (int j = obs_min.first; j < obs_max.first + 1; j++) {
                mymap.Obs.push_back(&mymap.point[i * mymap.Map_x + j]);
            }
        }
        mymap.StartNode = &mymap.point[start_point.second * mymap.Map_x + start_point.first];
        mymap.FinalNode = &mymap.point[final_point.second * mymap.Map_x + final_point.first];
        result = A_star(mymap);
        for (int i = 0; i < result.size(); i++) {
            std::cout << result[i].first << " " << result[i].second << std::endl;
        }
        
        rapidjson::Document path;
        path.SetObject();
        rapidjson::Value array1(rapidjson::kArrayType);
        rapidjson::Value array2(rapidjson::kArrayType);
        for (int i = 0; i < result.size(); i++) {
            array1.PushBack(result[i].first, path.GetAllocator());
            array2.PushBack(result[i].second, path.GetAllocator());
        }
 
        path.AddMember("x_path", array1, path.GetAllocator());
        path.AddMember("y_path", array2, path.GetAllocator());

        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        path.Accept(writer);

        std::string payload_2 = buffer.GetString();
        mqtt::message_ptr pub_msg = mqtt::make_message(TOPIC_2, payload_2);
        client_.publish(pub_msg);
              
    } 
};


int main() {
    mqtt::async_client client(SERVER_ADDRESS, ClIENT_ID);

    callback cb(client);
    client.set_callback(cb);

    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);

    try {
        client.connect(connOpts)->wait();
        client.subscribe(TOPIC,QOS);
        std::cout << "Subscribed to topic: " << TOPIC << std::endl;


        // Keep the subscriber running until interrupted
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
        client.unsubscribe(TOPIC)->wait();
        client.disconnect()->wait();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << "Error: " << exc.what() << std::endl;
        return 1;
    }

    return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
