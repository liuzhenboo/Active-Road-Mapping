#include <iostream>
#include <iostream>
class Cell
{
public:
    Cell();
    Cell(uint8_t class_state, float current_height);
    ~Cell();
    uint8_t Update(uint8_t class_state, float current_height);
    float GetZ(int id);
    uint8_t GetState();

private:
    // 格子的类别，０表示未探索，１表示是路，２表示是障碍物，３表示不确定
    uint8_t state;
    float height;
    int seen_times;
    // 对比一个格子不同点云的数量来判断路和障碍物

    int obs_seentimes;
    int ground_seentimes;
    float obs_height;
    float ground_height;
};