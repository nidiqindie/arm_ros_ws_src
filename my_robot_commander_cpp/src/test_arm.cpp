#include <QCoreApplication>
#include <iostream>
#include "hitrobot/hitbot_interface.h"
#include <unistd.h>

using namespace std;

void console_pause();
int test_initial();
int test_io();
int test_std_efg();
int test_movej();
int test_movel();


void console_pause() {
#ifdef WINDOWS
    system("pause");
#endif // WINDOWS

}

void Sleep(int ms){

#ifndef WINDOWS
    usleep(ms * 1000);

#endif

}


//------------------------------------------------------------ -
int robot_id = 3;
ControlBeanEx *robot;
int ret = 0;
int IO_COUNT = 11;
//------------------1，初始化网络服务器和手臂------------------
int test_initial() {
    net_port_initial();
    if (card_number_connect(robot_id)) {
        robot = get_robot(robot_id);
        ret = robot->initial(1, 210);
        robot->unlock_position();
        if (ret != 1) {
            cout << "initial ret = " << ret << endl;
            console_pause();
            return 0;
        }
    }
    else {
        cout << "robot isnot connected" << endl;
        console_pause();
    }
    return 1;
}

//------------------2，IO方面的测试---------------------------
int test_io() {
    for (int i = 0; i < IO_COUNT; i++) {
        robot->set_digital_out(i, 0);
    }
    Sleep(200);
    for (int i = 0; i < IO_COUNT; i++) {
        if (robot->get_digital_in(i) == 1) cout << "io_" << i << " test_1 ok" << endl;
        else {
            cout << "io_" << i << " test_1 ng" << endl;
            console_pause(); return 0;

        }
    }
    for (int i = 0; i < IO_COUNT; i++)
        robot->set_digital_out(i, 1);
    Sleep(200);
    for (int i = 0; i < IO_COUNT; i++) {
        if (robot->get_digital_in(i) == 0) cout << "io_" << i << " test_0 ok" << endl;
        else {
            cout << "io_" << i << " test_0 ng" << endl;
            console_pause(); return 0;
        }
    }
}
//------------------3，efg20 标准版本 无闭环校验---------------------------

int test_std_efg() {

    int efg_type = 0;
    float efg_dis = 0.0;
    robot->set_efg_state(20, 0);
    Sleep(1000);
    robot->get_efg_state(&efg_type, &efg_dis);
    cout << efg_type << "\t" << efg_dis << endl;

    robot->set_efg_state(20, 20);
    Sleep(1000);
    robot->get_efg_state(&efg_type, &efg_dis);
    cout << efg_type << "\t" << efg_dis << endl;
    return 1;
}

//------------------4，movej测试
int test_movej() {
    ret = robot->movej_angle(89, 89, 0, 0, 100, 0);
    ret = robot->movej_angle(-89,-89,-100,0,100,0);

    robot->movej_angle(-89, -89, 0, 0, 100, 0);
    robot->movej_xyz_lr(200, -200, 0, 0, 100, 0, -1);
    robot->movej_xyz_lr(200, 200, 0, 0, 100, 0, 1);
    robot->wait_stop();
    return 1;
}

//------------------5，movel测试
int test_movel() {

    robot->movel_xyz(200, 100, 0, 0, 100);  robot->wait_stop();
    robot->movel_xyz(200, -100, -100, 0, 100);  robot->wait_stop();
    return 1;

}

int std_loop(){
    /*
    robot->movej_xyz_lr(250,150,-25,0,100,0,1);robot->wait_stop();

    float sp_xy = 400;
    float sp_z = 100;
    float rough = 1;
    while(true){

        robot->movej_xyz_lr(250,150,0,0,sp_z,rough,1);
        robot->movej_xyz_lr(250,-150,0,0,sp_xy,rough,1);
        robot->movej_xyz_lr(250,-150,-25,0,sp_z,rough,1);

        robot->wait_stop();
        robot->movej_xyz_lr(250,-150,0,0,sp_z,rough,1);
        robot->movej_xyz_lr(250,150,0,0,sp_xy,rough,1);
        robot->movej_xyz_lr(250,150,-25,0,sp_z,rough,1);
        robot->wait_stop();
    }

*/

    float speed =400;

    while(true){
        robot->movej_xyz_lr(0,350,0,0,600,0,1);
        robot->wait_stop();
        robot->movej_xyz_lr(120,0,0,-90,speed,1,-1);
        robot->wait_stop();

        robot->movej_xyz_lr(0,-350,0,0,speed,1,-1);
        robot->wait_stop();
        //-----------------------------------------------------------------------------------------
        robot->movej_xyz_lr(0,350,0,0,600,0,1);
        robot->wait_stop();
        robot->movej_xyz_lr(120,0,0,-90,speed,1,-1);
        //robot->wait_stop();

        robot->movej_xyz_lr(0,-350,0,0,speed,1,-1);
        robot->wait_stop();

    }


}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    test_initial();
    test_io();
    //test_std_efg();
    //test_movej();
    //test_movel();
    console_pause();

    // std_loop();

    robot->wait_stop();


    while(true){
        Sleep(100);
        if(robot->is_connected()==false){
            cout<<"dis connected"<<endl;
        }
    }
    return a.exec();
}
