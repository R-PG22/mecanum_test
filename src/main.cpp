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
                }
            }
        }
        auto antiover = [](float val){return min(static_cast<int>(val * Motor_Max_Power), Motor_Max_Power);};
        penguin.pwm[0] = antiover(- rx_X + rx_Y - rx_R);
        penguin.pwm[1] = antiover(+ rx_X + rx_Y - rx_R);
        penguin.pwm[2] = antiover(- rx_X - rx_Y - rx_R);
        penguin.pwm[3] = antiover(+ rx_X - rx_Y - rx_R);

        penguin.send();
    }
}