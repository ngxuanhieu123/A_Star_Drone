#include <iostream>
#include <string>
#include <mqtt/client.h>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

const std::string MQTT_SERVER_ADDRESS = "tcp://42.96.40.234:1883";
const std::string MQTT_CLIENT_ID = "user";
const std::string MQTT_TOPIC = "user_set_up";
const std::string MQTT_TOPIC_2 = "algorithm_return";
const int QOS = 1;

class callback : public virtual mqtt::callback {
    void connection_lost(const std::string& cause) override {
        std::cout << "\nConnection lost" << std::endl;
        if (!cause.empty())
            std::cout << "\tCause: " << cause << std::endl;
    }

    void delivery_complete(mqtt::delivery_token_ptr token) override {
        std::cout << "\nDelivery complete" << std::endl;
    }

    void message_arrived(mqtt::const_message_ptr msg) override {
        std::cout << "Message arrived" << std::endl;
        std::cout << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
        std::cout << "\tpayload: '" << msg->to_string() << "'\n" << std::endl;
    }
};

int main() {
    mqtt::async_client client(MQTT_SERVER_ADDRESS, MQTT_CLIENT_ID);

    callback cb;
    client.set_callback(cb);

    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);

    try {
        client.connect(connOpts)->wait();

        rapidjson::Document message;
        message.SetObject();

        rapidjson::Value start("start", message.GetAllocator());
        rapidjson::Value s_point(rapidjson::kArrayType);

        s_point.PushBack(20.96137, message.GetAllocator());
        s_point.PushBack(105.74662, message.GetAllocator());
        message.AddMember(start, s_point, message.GetAllocator());
        
        
        rapidjson::Value final("final", message.GetAllocator());
        rapidjson::Value f_point(rapidjson::kArrayType);
        f_point.PushBack(20.96121, message.GetAllocator());
        f_point.PushBack(105.74653, message.GetAllocator());
        message.AddMember(final, f_point, message.GetAllocator());

        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        message.Accept(writer);

        std::string payload = buffer.GetString();

        mqtt::message_ptr pubmsg = mqtt::make_message(MQTT_TOPIC, payload);
        client.publish(pubmsg);

        std::cout << "Subscribing to topic '" << MQTT_TOPIC_2 << std::endl;
        client.subscribe(MQTT_TOPIC_2, QOS);
        std::cout << "Subscribed!" << std::endl;

        // Start the network loop
        client.start_consuming();

        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        client.disconnect();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << "Error: " << exc.what() << std::endl;
        return 1;
    }

    return 0;
}
