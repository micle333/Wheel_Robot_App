#include <ros/ros.h>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <optional>
#include <std_msgs/String.h>

template<typename MessageType>
class MessageWaiter {
public:
    MessageWaiter(ros::NodeHandle& nh, const std::string& topic, int queue_size = 1)
    : nh_(nh), received_(false), terminate_(false)  {
        subscriber_ = nh_.subscribe(topic, queue_size, &MessageWaiter::callback, this);
        std::cout << "Subscribed to " << topic << ". Waiting for msgs." << std::endl;
    }

    ~MessageWaiter() {
        terminate();
        subscriber_.shutdown(); // Отписываемся от топика
        std::cout << "Unsubscribed from topic and cleaned up." << std::endl;
    }

    std::optional<MessageType> waitForMessage(double timeout_seconds = -1) {
        std::cout << "Wait started." << std::endl;

        auto check_interval = std::chrono::milliseconds(10); // Интервал для проверки и вызова ros::spinOnce
        auto start_time = std::chrono::steady_clock::now();
        
        while (ros::ok()) {
            std::unique_lock<std::mutex> lock(mutex_);
            
            if (received_) {
                received_ = false;  // Сбрасываем флаг для следующего использования
                return message_;    // Возвращаем полученное сообщение
            }

            if (terminate_) {
                return std::nullopt; // Прерывание ожидания
            }

            if (timeout_seconds > 0) {
                auto elapsed = std::chrono::steady_clock::now() - start_time;
                if (elapsed > std::chrono::duration<double>(timeout_seconds)) {
                    return std::nullopt; // Таймаут
                }
            }

            cond_var_.wait_for(lock, check_interval);

            lock.unlock(); // Разблокируем мьютекс перед вызовом ros::spinOnce()
            ros::spinOnce(); // Обрабатываем входящие сообщения
        }

        return std::nullopt; // Прерывание ожидания при завершении ros::ok()
    }

    void terminate() {
        std::lock_guard<std::mutex> guard(mutex_);
        terminate_ = true;
        cond_var_.notify_all(); // Пробуждаем все потоки, которые могут быть заблокированы на ожидании
    }

private:
    ros::NodeHandle& nh_;
    ros::Subscriber subscriber_;
    MessageType message_;
    std::mutex mutex_;
    std::condition_variable cond_var_;
    bool received_;
    bool terminate_;

    void callback(const typename MessageType::ConstPtr& msg) {
        std::lock_guard<std::mutex> guard(mutex_);
        std::cout << "Got msg." << std::endl;
        if (!terminate_) {
            message_ = *msg;
            received_ = true;
            cond_var_.notify_one();
        }
    }
};