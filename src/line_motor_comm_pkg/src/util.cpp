#include "util.h"

namespace util
{
    // 函数来写入 q/dq 数据到 CSV 文件
    void writeToCSV(const std::string& filename, float data, std::string mark) {
        std::ofstream file;

        // 打开文件以追加模式
        file.open(filename, std::ios::app);

        if (!file.is_open()) {
            std::cerr << "无法打开文件 " << filename << std::endl;
            return;
        }

        // 将数据写入文件
        file << data << mark;

        file.close();
    }

    bool readdata(float* left_kuan, float* kneel, float* anklel, float* right_kuan, float* kneer, float* ankler ,const std::string& filename) {
        std::ifstream file(filename);
        std::string line;
        std::vector<std::string> column;

        if (file.is_open()) {
            int num = 0;
            while (std::getline(file, line)) {
                std::stringstream ss(line);
                std::string cell;
                for (int i = 0; std::getline(ss, cell, ','); ++i) {
                    switch(i) {
                        case 0:	// 第一列——left_angle_rad1
                        {	
                            left_kuan[num] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 1:	// left_line
                        {	
                            kneel[num] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 2:	// left_angle_rad3
                        {	
                            anklel[num] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 3:	// right_angle_rad1
                        {	
                            right_kuan[num] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 4:	// right_line
                        {	
                            kneer[num] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 5:	// right_angle_rad3
                        {	
                            ankler[num] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        default:
                            break;
                    }
                    
                }
                num++;
            }
            file.close();
        } else {
            std::cerr << "无法打开文件" << std::endl;
            return false;
        }
        return true;
        // 输出第一列的内容
        // for (const auto& value : column) {
        //     std::cout << value << std::endl;
        // }
    }

    void begin_gait(JointMotor& motorL_0_a, JointMotor& motorL_1_a, 
                    JointMotor& motorR_0_a, JointMotor& motorR_1_a, 
                    JointMotor& motorL_0, JointMotor& motorL_2, 
                    JointMotor& motorR_0, JointMotor& motorR_2, 
                    float motorL_0_ORI_a, float motorL_1_ORI_a, 
                    float motorR_0_ORI_a, float motorR_1_ORI_a, 
                    float motorL_0_ORI, float motorL_2_ORI, 
                    float motorR_0_ORI, float motorR_2_ORI, 
                    float* left_hip_begin, float* kneel_begin, 
                    float* anklel_begin, float* right_hip_begin, 
                    float* kneer_begin, float* ankler_begin, 
                    LineMotor& left_leg, LineMotor& right_leg, 
                    float* hip_roll, int unit_time, float k_ankle_hip_roll, int number_begin
                    ) {
        
        int TempCnt1 = 0;
        // 下蹲 + 暂停
        for (int i = 0 ; i < 600 ; i++) {
            motorL_0_a.PosMode(0.3, 10, motorL_0_ORI_a - anklel_begin[i], 3);
            motorL_1_a.PosMode(0.3, 10, motorL_1_ORI_a + anklel_begin[i], 3);
            motorR_0_a.PosMode(0.3, 10, motorR_0_ORI_a + ankler_begin[i], 4);
            motorR_1_a.PosMode(0.3, 10, motorR_1_ORI_a - ankler_begin[i], 4);
            motorR_2.PosMode(3, 1, motorR_2_ORI - right_hip_begin[i], 2);
            motorL_2.PosMode(3, 1, motorL_2_ORI + left_hip_begin[i], 1);
            if (i % 6 == 0) {
                TempCnt1 += 6;
            }
            // left_leg.Clear_PosCmd();
            left_leg.RelPos_Set(kneel_begin[TempCnt1], 2000);       //此处的RelPos已变成绝对位置模式
            // right_leg.Clear_PosCmd();
            right_leg.RelPos_Set(kneer_begin[TempCnt1], 2000);
            motorL_0.PosMode(0.5, 1, motorL_0_ORI, 1);//锁住侧摆关节
            motorR_0.PosMode(0.5, 1, motorR_0_ORI, 2);
        }

        int CntPitch = 600;       // 往前迈步时的数组元素位置
        // 由直立向左侧摆
        for(int i = 0 ; i < 0.5 * unit_time ; i++) {      //每次走0.5个unit_time
            motorL_0.PosMode(6, 10, motorL_0_ORI - hip_roll[i % (2 * unit_time)], 1);//“加”为王梦迪的
            motorR_0.PosMode(6, 10, motorR_0_ORI - hip_roll[i % (2 * unit_time)], 2);
            motorL_0_a.PosMode(1, 20, motorL_0_ORI_a - anklel_begin[CntPitch - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
            motorL_1_a.PosMode(1, 20, motorL_1_ORI_a + anklel_begin[CntPitch - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
            motorR_0_a.PosMode(1, 20, motorR_0_ORI_a + ankler_begin[CntPitch - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
            motorR_1_a.PosMode(1, 20, motorR_1_ORI_a - ankler_begin[CntPitch - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
        }
        
        int CntRoll = 0.5 * unit_time;
        int TempCnt2 = CntPitch;
        float V_LeftHip2, V_RightHip2, V_LeftAnkle, V_RightAnkle;
        float Kp = 1;
	    float Kd = 10;
        // 右腿迈半步
        for(int i = CntPitch ; i < number_begin ; i++) {  
            if (i < number_begin - 1) {
				V_LeftHip2 = (left_hip_begin[i + 1] - left_hip_begin[i]) / 0.03;    // 一个周期大概0.03s，之后增快后需要调整
				V_RightHip2 = (right_hip_begin[i + 1] - right_hip_begin[i]) / 0.03;
				V_LeftAnkle = (anklel_begin[i + 1] - anklel_begin[i]) / 0.03;
				V_RightAnkle = (ankler_begin[i + 1] - ankler_begin[i]) / 0.03;
			}
			else {
				V_LeftHip2 = 0;
				V_RightHip2= 0;	
				V_LeftAnkle = 0;	
				V_RightAnkle = 0;
			}

            motorL_0_a.PosMode(Kp, Kd, motorL_0_ORI_a - anklel_begin[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 3);
			// motorL_1_a.MixedMode(0,V_LeftAnkle,motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],8 ,Kp,  Kd);			// motorL_1_a.MixedMode(0,V_LeftAnkle,motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],8 ,30,  3);
            motorL_1_a.PosMode(Kp, Kd, motorL_1_ORI_a + anklel_begin[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 3);
			motorR_0_a.MixedMode(0, V_RightAnkle, motorR_0_ORI_a + ankler_begin[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], Kp, Kd, 4);		// motorR_0_a.MixedMode(0,V_RightAnkle,motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],3, 30,  4);
            // motorR_0_a.PosMode(0.6, 10, motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
            motorR_1_a.MixedMode(0, -V_RightAnkle, motorR_1_ORI_a - ankler_begin[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], Kp, Kd, 4);		// motorR_1_a.MixedMode(0,-V_RightAnkle,motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],3, 30,  4);
            // motorR_1_a.PosMode(0.6, 10, motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)], 4);
			// motorR_2.PosMode(1, 10, motorR_2_ORI-right_kuan[CntPitch], 2);
            motorR_2.MixedMode(0, -V_RightHip2 ,motorR_2_ORI - right_hip_begin[i], 3, 30,  2);  //之前刚度是3，阻尼是30
            // motorL_2.PosMode(1, 10, motorL_2_ORI+left_kuan[CntPitch], 1);
            motorL_2.MixedMode(0, V_LeftHip2, motorL_2_ORI + left_hip_begin[i], 3, 30, 1);

            if (i % 6 == 0) {	
                TempCnt2 += 6;
			}
			std::cout << "TempCnt2:   " << TempCnt2 << std::endl;
            left_leg.Clear_PosCmd();
			left_leg.RelPos_Set(kneel_begin[TempCnt2], 500);
            right_leg.Clear_PosCmd();
			right_leg.RelPos_Set(kneer_begin[TempCnt2], 500);
        }

        for(int i = 0.5 * unit_time ; i < unit_time ; i++) {      // 侧摆回正
            motorL_0.PosMode(6, 10, motorL_0_ORI - hip_roll[i % (2 * unit_time)], 1);//“加”为王梦迪的
            motorR_0.PosMode(6, 10, motorR_0_ORI - hip_roll[i % (2 * unit_time)], 2);
            motorL_0_a.PosMode(1, 20, motorL_0_ORI_a - anklel_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
            motorL_1_a.PosMode(1, 20, motorL_1_ORI_a + anklel_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
            motorR_0_a.PosMode(1, 20, motorR_0_ORI_a + ankler_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
            motorR_1_a.PosMode(1, 20, motorR_1_ORI_a - ankler_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
        }
    }

    void period_gait(JointMotor& motorL_0_a, JointMotor& motorL_1_a, 
                    JointMotor& motorR_0_a, JointMotor& motorR_1_a, 
                    JointMotor& motorL_0, JointMotor& motorL_2, 
                    JointMotor& motorR_0, JointMotor& motorR_2, 
                    float motorL_0_ORI_a, float motorL_1_ORI_a, 
                    float motorR_0_ORI_a, float motorR_1_ORI_a, 
                    float motorL_0_ORI, float motorL_2_ORI, 
                    float motorR_0_ORI, float motorR_2_ORI, 
                    float* left_hip_period, float* kneel_period, 
                    float* anklel_period, float* right_hip_period, 
                    float* kneer_period, float* ankler_period, 
                    float* ankler_begin, float* anklel_begin,
                    LineMotor& left_leg, LineMotor& right_leg, 
                    float* hip_roll, int unit_time, float k_ankle_hip_roll, int number_begin, int number_period, int loop_num
                    ) {
        float V_LeftHip2, V_RightHip2, V_LeftAnkle, V_RightAnkle;
        for (int i = 0 ; i < loop_num ; i++) {
            // 由直立侧摆向右
            for(int i = unit_time ; i < 1.5 * unit_time ; i++) {      
                motorL_0.PosMode(6, 10, motorL_0_ORI - hip_roll[i % (2 * unit_time)], 1);
                motorR_0.PosMode(6, 10, motorR_0_ORI - hip_roll[i % (2 * unit_time)], 2);
                motorL_0_a.PosMode(1, 20, motorL_0_ORI_a - anklel_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
                motorL_1_a.PosMode(1, 20, motorL_1_ORI_a + anklel_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
                motorR_0_a.PosMode(1, 20, motorR_0_ORI_a + ankler_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
                motorR_1_a.PosMode(1, 20, motorR_1_ORI_a - ankler_begin[number_begin - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
            }

            int CntRoll = 1.5 * unit_time;
            int TempCnt2 = 0;
            float Kp = 1;
	        float Kd = 10;
            // 迈左腿
            for(int i = 0 ; i < number_period / 2 ; i++) {
                // 粗略计算期望速度
                if (i < number_period / 2 - 1) {
                    V_LeftHip2 = (left_hip_period[i + 1] - left_hip_period[i]) / 0.03;    // 一个周期大概0.03s，之后增快后需要调整
                    V_RightHip2 = (right_hip_period[i + 1] - right_hip_period[i]) / 0.03;
                    V_LeftAnkle = (anklel_period[i + 1] - anklel_period[i]) / 0.03;
                    V_RightAnkle = (ankler_period[i + 1] - ankler_period[i]) / 0.03;
                }
                else {
                    V_LeftHip2 = 0;
                    V_RightHip2= 0;	
                    V_LeftAnkle = 0;	
                    V_RightAnkle = 0;
                }

                motorL_0_a.PosMode(Kp, Kd, motorL_0_ORI_a - anklel_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 3);  //手扶用0.6
                // motorL_1_a.MixedMode(0,V_LeftAnkle,motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  3);
				motorL_1_a.PosMode(Kp, Kd, motorL_1_ORI_a + anklel_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 3);
				// motorR_0_a.MixedMode(0,V_RightAnkle,motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  4);
				motorR_0_a.PosMode(Kp, Kd, motorR_0_ORI_a + ankler_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 4);
				// motorR_1_a.MixedMode(0,-V_RightAnkle,motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  4);
				motorR_1_a.PosMode(Kp, Kd, motorR_1_ORI_a - ankler_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 4);
				motorR_2.PosMode(3, 30, motorR_2_ORI - right_hip_period[i], 2); //手扶用0.5
				// motorR_2.MixedMode(0,-V_RightHip2,motorR_2_ORI-right_kuan[CntPitch],0.5, 30,  2);
				motorL_2.PosMode(3, 30, motorL_2_ORI + left_hip_period[i], 1);

                if (i % 6 == 0) {	
                    TempCnt2 += 6;
                }
                std::cout << "TempCnt2:   " << TempCnt2 << std::endl;
                left_leg.Clear_PosCmd();
                left_leg.RelPos_Set(kneel_period[TempCnt2], 500);
                right_leg.Clear_PosCmd();
                right_leg.RelPos_Set(kneer_period[TempCnt2], 500);
            }

            // 由右侧向左侧摆至左侧
            for(int i = 1.5 * unit_time ; i < 2.5 * unit_time ; i++) {      
                motorL_0.PosMode(6, 10, motorL_0_ORI - hip_roll[i % (2 * unit_time)], 1);
                motorR_0.PosMode(6, 10, motorR_0_ORI - hip_roll[i % (2 * unit_time)], 2);
                motorL_0_a.PosMode(1, 20, motorL_0_ORI_a - anklel_period[number_period / 2 - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
                motorL_1_a.PosMode(1, 20, motorL_1_ORI_a + anklel_period[number_period / 2 - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
                motorR_0_a.PosMode(1, 20, motorR_0_ORI_a + ankler_period[number_period / 2 - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
                motorR_1_a.PosMode(1, 20, motorR_1_ORI_a - ankler_period[number_period / 2 - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
            }

            CntRoll = 0.5 * unit_time;
            TempCnt2 = number_period / 2;
            // 迈右腿
            for(int i = number_period / 2 ; i < number_period ; i++) {
                // 粗略计算期望速度
                if (i < number_period - 1) {
                    V_LeftHip2 = (left_hip_period[i + 1] - left_hip_period[i]) / 0.03;    // 一个周期大概0.03s，之后增快后需要调整
                    V_RightHip2 = (right_hip_period[i + 1] - right_hip_period[i]) / 0.03;
                    V_LeftAnkle = (anklel_period[i + 1] - anklel_period[i]) / 0.03;
                    V_RightAnkle = (ankler_period[i + 1] - ankler_period[i]) / 0.03;
                }
                else {
                    V_LeftHip2 = 0;
                    V_RightHip2= 0;	
                    V_LeftAnkle = 0;	
                    V_RightAnkle = 0;
                }

                motorL_0_a.PosMode(Kp, Kd, motorL_0_ORI_a - anklel_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 3);  //手扶用0.6
                // motorL_1_a.MixedMode(0,V_LeftAnkle,motorL_1_ORI_a+anklel[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  3);
				motorL_1_a.PosMode(Kp, Kd, motorL_1_ORI_a + anklel_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 3);
				// motorR_0_a.MixedMode(0,V_RightAnkle,motorR_0_ORI_a+ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  4);
				motorR_0_a.PosMode(Kp, Kd, motorR_0_ORI_a + ankler_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 4);
				// motorR_1_a.MixedMode(0,-V_RightAnkle,motorR_1_ORI_a-ankler[CntPitch]+k_ankle_hip_roll*hip_roll[CntRoll%(2*unit_time)],0.5, 20,  4);
				motorR_1_a.PosMode(Kp, Kd, motorR_1_ORI_a - ankler_period[i] + k_ankle_hip_roll * hip_roll[CntRoll % (2 * unit_time)], 4);
				motorR_2.PosMode(3, 30, motorR_2_ORI - right_hip_period[i], 2); //手扶用0.5
				// motorR_2.MixedMode(0,-V_RightHip2,motorR_2_ORI-right_kuan[CntPitch],0.5, 30,  2);
				motorL_2.PosMode(3, 30, motorL_2_ORI + left_hip_period[i], 1);

                if (i % 6 == 0) {	
                    TempCnt2 += 6;
                }
                std::cout << "TempCnt2:   " << TempCnt2 << std::endl;
                left_leg.Clear_PosCmd();
                left_leg.RelPos_Set(kneel_period[TempCnt2], 500);
                right_leg.Clear_PosCmd();
                right_leg.RelPos_Set(kneer_period[TempCnt2], 500);
            }

            // 由左侧向右侧摆至直立
            for(int i = 0.5 * unit_time ; i < unit_time ; i++) {      
                motorL_0.PosMode(6, 10, motorL_0_ORI - hip_roll[i % (2 * unit_time)], 1);
                motorR_0.PosMode(6, 10, motorR_0_ORI - hip_roll[i % (2 * unit_time)], 2);
                motorL_0_a.PosMode(1, 20, motorL_0_ORI_a - anklel_period[number_period - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
                motorL_1_a.PosMode(1, 20, motorL_1_ORI_a + anklel_period[number_period - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 3);
                motorR_0_a.PosMode(1, 20, motorR_0_ORI_a + ankler_period[number_period - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
                motorR_1_a.PosMode(1, 20, motorR_1_ORI_a - ankler_period[number_period - 1] + k_ankle_hip_roll * hip_roll[i % (2 * unit_time)], 4);
            }
        }
    }

    int readdatafromcsv(float** left_hip_roll, float** left_pitch, float** left_knee, float** left_ankle, float** left_foot, float** right_hip_roll, float** right_pitch, float** right_knee, float** right_ankle, float** right_foot, const std::string& filename) {
        
        // 注意此函数读取的文件第一行是字符串名字
        std::ifstream file(filename);
        std::string line;
        std::vector<std::string> column;

        // 读取文件行数
        int line_count = 0;
        // 确保文件被成功打开
        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)) {
                line_count++; // 对每一行进行计数
            }
            std::cout<<"文件行数: \t"<<line_count<<std::endl;
            file.close();
        } else {
            std::cerr << "无法打开文件" << std::endl;
        }

        // 给数组分配存储空间
        *left_hip_roll = new float[line_count-1];
        *left_pitch = new float[line_count-1];
        *left_knee = new float[line_count-1];
        *left_ankle = new float[line_count-1];
        *left_foot = new float[line_count-1];
        *right_hip_roll = new float[line_count-1];
        *right_pitch = new float[line_count-1];
        *right_knee = new float[line_count-1];
        *right_ankle = new float[line_count-1];
        *right_foot = new float[line_count-1];

        std::ifstream file_getdata(filename);
        if (file_getdata.is_open()) {
            int num = 0;
            while (std::getline(file_getdata, line)) {
                std::stringstream ss(line);
                std::string cell;
                if(num==0) 
                {
                    num ++;
                    continue;
                }
                for (int i = 0; std::getline(ss, cell, ','); ++i) {
                    switch(i) {
                        case 0:	// 第一列——left_angle_rad1
                        {	
                            (*left_hip_roll)[num-1] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 1:	// left_line
                        {	
                            (*left_pitch)[num-1] = std::stof(cell);
                            (*left_pitch)[num-1] = -(*left_pitch)[num-1];
                            break; // 仅获取第二列后退出循环
                        }
                        case 2:	// left_angle_rad3
                        {	
                            (*left_knee)[num-1] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 3:	// right_angle_rad1
                        {	
                            (*left_ankle)[num-1] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 4:	// right_line
                        {	
                            (*left_foot)[num-1] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 5:	// 第一列——left_angle_rad1
                        {	
                            (*right_hip_roll)[num-1] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 6:	// left_line
                        {	
                            (*right_pitch)[num-1] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 7:	// left_angle_rad3
                        {	
                            (*right_knee)[num-1] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 8:	// right_angle_rad1
                        {	
                            (*right_ankle)[num-1] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        case 9:	// right_line
                        {	
                            (*right_foot)[num-1] = std::stof(cell);
                            break; // 仅获取第二列后退出循环
                        }
                        default:
                            break;
                    }
                    
                }
                num++;
            }
            file_getdata.close();
        } else {
            std::cerr << "无法打开文件" << std::endl;
            return 0;
        }
        std::cout<<(*right_knee)[line_count-4]<<std::endl;
        std::cout<<(*right_knee)[line_count-3]<<std::endl;
        return line_count - 1;
        // 输出第一列的内容
        // for (const auto& value : column) {
        //     std::cout << value << std::endl;
        // }
    }

}