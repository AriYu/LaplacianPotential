#include <iostream>
#include <vector>

#include "../include/LaplacianPotential.h"

#define MAPSIZE_X 50
#define MAPSIZE_Y 50

#define P0 1.0 // 障害物を領域内に含む格子点のポテンシャルの初期値

using namespace std;

int main(void)
{
    // ポテンシャル場を定義
    PotentialField map(MAPSIZE_X, MAPSIZE_Y);

    // スタートとゴールを設定
    Position start = { 15, 10 }; // 15 25
    Position goal  = { 45, 15 };

    // ポテンシャルを初期化
    //map.Reset(P0); // 自由領域に属する格子点のポテンシャルの初期値は障害物の初期値と同じにする
    map.Reset(P0);    
    // ゴールを設定（ゴールのポテンシャルは0）
    map.SetValue(goal.x, goal.y, 0.0, true);
    //map.SetValue(7, 45, 0.0, true);

    // 障害物を設定（障害物のポテンシャルは正の定数）


    // 障害物パターン1------------------------------------------------------
    // for(unsigned int x = 15; x <= 35; x++){
    //     for(unsigned int y = 15; y <= 35; y++){
    //         map.SetValue(x, y , P0, true);
    //     }
    // }
    // ----------------------------------------------------------------------

    // 障害物パターン2------------------------------------------------------
    // for(int i=10; i<=40; i++){
    //     map.SetValue(10,i,1,true);
    //  }
    // for(int i=10; i<=40; i++){
    //     map.SetValue(40,i,1,true);
    //  }
    // for(int i=10; i<=40; i++){
    //     map.SetValue(i,10,1,true);
    //  }
    // ----------------------------------------------------------------------

    // 障害物パターン3------------------------------------------------------
    for(int i=0; i<=10; i++){
        map.SetValue(i,35,P0,true);
    }
    for(int i=15; i<=35; i++){
        map.SetValue(10,i,P0,true);
    }
    for(int i=10; i<=15; i++){
        map.SetValue(i,15,P0,true);
    }
    for(int i=0; i<=5; i++){
        map.SetValue(i,45,P0,true);
    }
    for(int i=0; i<=25; i++){
        map.SetValue(30,i,P0,true);
    }
    for(int i=20; i<=30; i++){
        map.SetValue(i,25,P0,true);
    }
    for(int i=25; i<=45; i++){
        map.SetValue(20,i,P0,true);
    }
    for(int i=10; i<=20; i++){
        map.SetValue(i,45,P0,true);
    }
    for(int i=40; i<=49; i++){
        map.SetValue(i,25,P0,true);
    }
    for(int i=20; i<=40; i++){
        map.SetValue(i,35,P0,true);
    }
    // ----------------------------------------------------------------------


    // ラプラスポテンシャルを計算する。
    unsigned int laplacian_loop      = 1000; 
    double       laplacian_threshold = (1e-15); // not using
    int i = 1;
    do{
        cout << i << endl;
        map.CalculateLaplacianPotential(laplacian_loop, laplacian_threshold);
        i++;
        if(i >= 1000){ break;}
        //cout << map.isOK(goal) << endl;
    }while(map.isOK(start) != 0);

    // 経路探索を行う。
    map.CreatePath(start, goal);
  
    // 生成されたポテンシャル場を保存する。
    map.SavePotentialField("./data/potential_field2.dat");
    map.SaveObstaclePotentialField("./data/obstacle_potential_field.dat");

    // 生成された経路を保存する。
    map.SavePath("./data/path.dat");

    return 0;
}
