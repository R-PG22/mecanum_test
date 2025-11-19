#include <map>
#include "mbed.h"
#include "FIRSTPENGUIN.hpp"

BufferedSerial esp(PA_9, PA_10, 115200);
BufferedSerial pc(USBTX, USBRX, 115200);
CAN can(PA_11, PA_12, (int)1e6);
constexpr uint32_t can_id = 35; // FPのCAN通信のIDを設定
FirstPenguin penguin(can_id, can); // FPの設定

constexpr int Motor_Max_Power = 10000;
float rx_X = 0;
float rx_Y = 0;
float rx_R = 0;
std::map<int, std::pair<std::string, bool>> buttons = {
    {0, {"Up", false}},
    {1, {"Down", false}},
    {2, {"Right", false}},
    {3, {"Left", false}},
    {4, {"Tri", false}},
    {5, {"Cir", false}},
    {6, {"Cro", false}},
    {7, {"Squ", false}},
    {8, {"L1", false}},
    {9, {"R1", false}},
    {10, {"L2", false}},
    {11, {"R2", false}},
};

bool isnum(const char *c){
    // 引数のchar配列が数字のみか調べる
    if (c == nullptr || *c == '\0') return 0;

    for (int i = 0; c[i] != '\0'; i++){
        if (!isdigit(c[i])) return 0;
    }
    return 1;
};
float adjust(const int pos_value){
    // 座標を0以上1以下にして、デッドスポットをつける
    float adj_val = pos_value / 127;
    if (abs(adj_val) < 0.1) adj_val = 0;
    return adj_val;
}

int main(){
    char buf[64];
    int buf_index = 0;

    while(1){
        if (esp.readable()){
            char c;
            int len = esp.read(&c, 1);
            if (len > 0){
                // シリアルで来るコマンド:
                // 'x' ... X軸値の終端
                // 'y' ... Y軸値の終端
                // 'r' ... R軸値の終端
                // 'b' ... その直後に17文字の '0'/'1' が来る（ボタン状態）
                if (c == 'x'){
                    if (buf_index > 0){
                        buf[buf_index] = '\0';
                        buf_index = 0;
                        if (isnum(buf)) rx_X = adjust(stoi(buf));
                    }
                }else if (c == 'y'){
                    if (buf_index > 0){
                        buf[buf_index] = '\0';
                        buf_index = 0;
                        if (isnum(buf)) rx_Y = adjust(stoi(buf));
                    }
                }else if (c == 'r'){
                    if (buf_index > 0){
                        buf[buf_index] = '\0';
                        buf_index = 0;
                        if (isnum(buf)) rx_R = adjust(stoi(buf));
                    }
                }else if (c == 'b'){
                    // ボタン状態の更新
                    for(auto &b : buttons){
                        b.second.second = buf[b.first] == '1' ? true : false;
                    }
                }
                else{
                    buf[buf_index++] = c;
                    if (buf_index >= sizeof(buf) - 1){
                        buf_index = 0; // バッファオーバーフロー防止
                    }
                }
            }
        auto antiover = [](float val){return min(static_cast<int>(val * Motor_Max_Power), Motor_Max_Power);};
        penguin.pwm[0] = antiover(- rx_X + rx_Y - rx_R);
        penguin.pwm[1] = antiover(+ rx_X + rx_Y - rx_R);
        penguin.pwm[2] = antiover(- rx_X - rx_Y - rx_R);
        penguin.pwm[3] = antiover(+ rx_X - rx_Y - rx_R);

        printf("%d %d %d %d\n",penguin.pwm[0],penguin.pwm[1],penguin.pwm[2],penguin.pwm[3]);

        penguin.send();
        }
    }
}