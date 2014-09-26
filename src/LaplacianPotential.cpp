#include "../include/LaplacianPotential.h"

using namespace std;

PotentialField::PotentialField()
{
    m_xsize = 0;
    m_ysize = 0;
    obsValue = 0;
}

PotentialField::PotentialField(unsigned int xsize, unsigned int ysize)
{
    m_xsize = xsize;
    m_ysize = ysize;
    potential_field.resize(m_xsize);
    size_t size = potential_field.size();
    for(size_t x = 0; x < size; x++){
        potential_field[x].resize(m_ysize);
    }
}

int PotentialField::Reset(double value)
{
    obsValue = value;

    for(unsigned int x = 0; x < m_xsize; x++){
        for(unsigned int y = 0; y < m_ysize; y++){
            potential_field[x][y].value = value;
            potential_field[x][y].obs = false;
        }
    }

    // 壁は障害物とする
    for(unsigned int x = 0; x < m_xsize; x++){
        SetValue(x, 0, value, true);
        SetValue(x, m_ysize-1, value, true);
    }
    for(unsigned int y = 0; y < m_ysize; y++){
        SetValue(0, y, value, true);
        SetValue(m_xsize-1, y, value, true);
    }

    return 0;
}

int PotentialField::SetValue(unsigned int x, unsigned int y, double value, bool isObs)
{
    if( (m_xsize < x) ){
        cout << "Out of range in x : " << x << endl;
        return -1;
    }
    if( (m_ysize < y) ){
        cout << "Out of range in y : " << y << endl;
        return -1;
    }
    potential_field[x][y].value = value;
    potential_field[x][y].obs = isObs;
    
    return 0;

}

double PotentialField::GetValue(unsigned int x, unsigned int y)
{
    if( (m_xsize < x) ){
        cout << "Out of range in x : " << x << endl;
        return 0;
    }
    if( (m_ysize < y) ){
        cout << "Out of range in y : " << y << endl;
        return 0;
    }
    return potential_field[x][y].value;
}

int PotentialField::SavePotentialField(string file_name)
{
    fstream output(file_name.c_str(), ios::out);
    
    for(unsigned int x = 0; x < m_xsize; x++){
        for(unsigned int y = 0; y < m_ysize; y++){
            output << x << " " << y << " " << fixed << potential_field[x][y].value << endl;
        }
        output << endl;
    }
    output.close();
    return 0;
}

int PotentialField::SaveObstaclePotentialField(string file_name)
{
    fstream output(file_name.c_str(), ios::out);
    
    for(unsigned int x = 0; x < m_xsize; x++){
        for(unsigned int y = 0; y < m_ysize; y++){
            if(potential_field[x][y].obs){
                output << x << " " << y << " " << fixed << potential_field[x][y].value << endl;
            }
        }
        output << endl;
    }
    output.close();
    return 0;
}

int PotentialField::CalculateLaplacianPotential(unsigned int numofloop, double thres)
{
    cout << "CalculateLaplacianPotential() is called" << endl;

    double tmp, sum;
    unsigned int i = 1;

    //for(unsigned int loop = 0; loop < numofloop; loop++){
    while(1){  
        sum = 0;
        for(unsigned int x = 1; x < m_xsize-1; x++){
            for(unsigned int y = 1; y < m_ysize-1; y++){
                //cout << "x : " << x << "\ty:" << y << "\tobs :" << potential_field[x][y].obs << endl;
                tmp = potential_field[x][y].value;
                if(!potential_field[x][y].obs){
                    potential_field[x][y].value = (potential_field[x][y-1].value 
                                                   + potential_field[x][y+1].value
                                                   + potential_field[x-1][y].value
                                                   + potential_field[x+1][y].value)/4.0;
                }
                sum+=fabs(tmp - potential_field[x][y].value);
            }
        }
        i++;
        if((sum/(m_xsize*m_ysize)) < thres && i > numofloop){
            break;
        }
    }
    cout << i << endl;
    return 0;
}

int PotentialField::isOK(Position start)
{
    // スタートの8近傍でポテンシャル場が変わっていたら0を返す
    for(int x = -1; x <= 1; x++){
        for(int y = -1; y <= 1; y++){
            if( x == 0 && y == 0 ){
                continue;
            }else{
                if( obsValue != GetValue(start.x + x, start.y + y)){
                    cout << GetValue(start.x + x, start.y + y) << endl;
                    return 0;
                }
            }
        }
    }
    return -1;
}

int PotentialField::CreatePath(Position start, Position goal)
{
    cout << "CreatePath() is called" << endl;
    Position robot = start;
    Position next;
    double dist = 0;
    int loop = 0;
    double  minValue;

    do{
        path.push_back(robot);
        path_potential.push_back(potential_field[robot.x][robot.y].value);
        minValue = potential_field[robot.x][robot.y].value;
        next.x = 0; next.y = 0;
        // 8近傍で最もポテンシャルが低い方を探す
        for(int x = -1; x <= 1; x++){
            for(int y = -1; y <= 1; y++){
                if( x == 0 && y == 0 ){
                    continue;
                }else{
                    if(minValue > GetValue(robot.x + x, robot.y + y)){
                        minValue = GetValue(robot.x + x, robot.y + y);
                        next.x = x; next.y = y;
                    }
                }
            }
        }
        // 8近傍のポテンシャルが同じで経路が見つからなかった場合、一番近くなる方向に行く。
        if(next.x == 0 && next.y == 0){
            minValue = sqrt(pow(goal.x - robot.x, 2) + pow(goal.y - robot.y, 2));
            for(int x = -1; x <= 1; x++){
                for(int y = -1; y <= 1; y++){
                    if( x == 0 && y == 0 ){
                        continue;
                    }else{
                        if(minValue >= sqrt(pow(goal.x - (robot.x+x), 2) + pow(goal.y - (robot.y+y), 2))){
                            if(potential_field[robot.x + x][robot.y + y].obs){
                                continue;
                            }
                            minValue = sqrt(pow(goal.x - (robot.x+x), 2) + pow(goal.y - (robot.y+y), 2));
                            next.x = x; next.y = y;
                        }
                    }
                }
            }            
        }
        robot.x = robot.x + next.x;
        robot.y = robot.y + next.y;
        dist = sqrt(pow(robot.x - goal.x, 2) + pow(robot.y - goal.y, 2));
        loop++;
        if(loop > 100){
            return -1;
        }
    }while(dist >= 0.1);

    return 0;
}

int PotentialField::SavePath(std::string file_name)
{
    fstream output(file_name.c_str(), ios::out);
    size_t size = path.size();    

    for(unsigned int i = 0; i < size; i++){
        output << path[i].x << " " << path[i].y << " " << path_potential[i] << endl;
    }
    output.close();
    return 0;
}
