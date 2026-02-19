#include "SDclass.h"

extern bool set_print;

//ファイル名の候補0~4の5通りある．
//実際に入れるファイルの名前は/fs/を抜いた部分
char path_0[20] = "/fs/PATH.txt";
char vel_0[20] = "/fs/VEL.txt";

char path_1[20] = "/fs/PATH_1.txt";
char vel_1[20] = "/fs/VEL_1.txt";

char path_2[20] = "/fs/PATH_2.txt";
char vel_2[20] = "/fs/VEL_2.txt";

char path_3[20] = "/fs/PATH_3.txt";
char vel_3[20] = "/fs/VEL_3.txt";

char path_4[20] = "/fs/PATH_TEST.txt";
char vel_4[20] = "/fs/VEL_TEST.txt";

FATFileSystem fs("fs");
SDBlockDevice sd(P8_5, P8_6, P8_3, P8_4);
mySDclass::mySDclass() {}

/* SDカードの初期化 */
int mySDclass::init()
{
    if (fs.mount(&sd) == 0) {
        sd.frequency(2000000);
        SD_enable = true;
        if(set_print) printf("[INFO] mySDclass init done\n");
        return 0;
    }

    if(set_print) printf("[INFO] mySDclass init failure\n");
    return -1;
}

/* ログデータ書き出し用ファイル名の設定 */
int mySDclass::make_logfile()
{
    // bool nameOK = false;
    int file_num = 0;

    DIR *d;
    d = opendir("/fs");
    if (d != NULL) {
        while (readdir(d) != NULL) {
            file_num++;
        }
    }

    int keta = 0;
    for(int tmp = file_num; (tmp /= 10) != 0; keta++);
    logFileName = "/fs/LOG_R2_ver1_";
    for(int i = 0; i < (2 - keta); i++) {
        logFileName += "0";
    }
    logFileName += to_string(file_num);
    logFileName += ".txt";
    //c_logFileName = logFileName.c_str();

    return file_num;
}

/* ログデータ書き出し用の関数 */
int mySDclass::write_logdata(string dataString)
{

    if(dataFile == 0) {
        dataFile = fopen(logFileName.c_str(), "w");
        fclose(dataFile);
        fopen(logFileName.c_str(), "a");
    }

    if(dataFile) {
        fprintf(dataFile, dataString.c_str());
        fflush(dataFile);
        return 0;
    } else {
        if(set_print) printf("[INFO] error to open log file\n");
        return -1;
    }
}

void mySDclass::close_logdata(){
    fclose(dataFile);
}

int mySDclass::path_read(int fileNum, double Px[], double Py[], double vel[],
                         double angle[], int mode[], double acc_param[],
                         double dec_param[])
{
    FILE *myFile;
    char *pathFile;
    char *velFile;
    char tmpchar;
    char tmpA[10], tmpB[10], tmpC[10], tmpD[10], tmpE[10];
    bool file_end = false;
    int numa = 0, numb = 0, numc = 0, numd = 0, nume = 0;
    int path_num = 0, point_num = 0;

    switch(fileNum){    //ファイル名の設定<--------------------
        case 0:
            pathFile = path_0;
            velFile = vel_0;
            break;
        case 1:
            pathFile = path_1;
            velFile = vel_1;
            break;
        case 2:
            pathFile = path_2;
            velFile = vel_2;
            break;
        case 3:
            pathFile = path_3;
            velFile = vel_3;
            break;
        case 4:
            pathFile = path_4;
            velFile = vel_4;
            break;
        defalut:
            return -1;
    }

    // パス設定用ファイルからデータを読み込む
    if(set_print) printf("openning... %s\n", pathFile);
    myFile = fopen(pathFile, "r");

    if (myFile) {
        // read from the file until there's nothing else in it:
        while (!file_end && !feof(myFile)) {
            while ((tmpchar = fgetc(myFile)) != ',') { // カンマが来るまで繰り返し
                tmpA[numa] = tmpchar; // 文字列に1文字ずつ格納していく
                numa++;
            }
            *Px = str2double(tmpA, numa); //関数でdoubleに変換
            // Serial.print(tmpA);
            if(set_print) printf("%lf, ", *Px);
            for (int i = 0; i < 10; i++)
                tmpA[i] = 0; // 文字列を初期化
            numa = 0;
            Px++;
            while (((tmpchar = fgetc(myFile)) != '\r' && tmpchar != ';') && tmpchar != '/') { // 改行コードかセミコロン，スラッシュが来るまで繰り返し
                tmpB[numb] = tmpchar;
                numb++;
            }
            if (tmpchar == ';') {
                file_end = true;
            } else if (tmpchar == '/') { // コメントアウト対応
                while ((tmpchar = fgetc(myFile)) != '\n');
            } else {
                fgetc(myFile); // "\n"を捨てるため
            }
            *Py = str2double(tmpB, numb); //関数でdoubleに変換
            point_num++;

            if(set_print) printf("%lf\n", *Py);
            // Serial.println(tmpB);
            for (int i = 0; i < 10; i++)
                tmpB[i] = 0;
            numb = 0;
            Py++;
        }
        // Serial.print("path done! ");
        file_end = false;
        //   the file:
        fclose(myFile);
    } else {
        // if the file didn't open, print an error:
        // Serial.println("error opening test.txt");

        if(set_print) printf("didn't open path file");
        return -2;
    }
    if(set_print) printf("point num: %d\n", point_num);

    // 速度設定用ファイルからデータを読み込む
    if(set_print) printf("openning... %s\n", velFile);
    myFile = fopen(velFile, "r");
    if (myFile) {
        while (!file_end && !feof(myFile)) {
            while ((tmpchar = fgetc(myFile)) != ',') {
                tmpA[numa] = tmpchar;
                numa++;
            }
            *vel = str2double(tmpA, numa); //関数でdoubleに変換
            // Serial.print(tmpA);
            if(set_print) printf("%lf, ", *vel);
            for (int i = 0; i < 10; i++)
                tmpA[i] = 0;
            numa = 0;
            vel++;
            //////////////////////////////////
            while ((tmpchar = fgetc(myFile)) != ',') {
                tmpB[numb] = tmpchar;
                numb++;
            }
            *angle = str2double(tmpB, numb); //関数でdoubleに変換
            // Serial.print(tmpA);
            if(set_print) printf("%lf, ", *angle);
            for (int i = 0; i < 10; i++)
                tmpB[i] = 0;
            numb = 0;
            angle++;
            //////////////////////////////////
            while ((tmpchar = fgetc(myFile)) != ',') {
                tmpC[numc] = tmpchar;
                numc++;
            }
            *mode = str2uint(tmpC, numc); //関数でdoubleに変換
            // Serial.print(tmpA);
            if(set_print) printf("%d, ", *mode);
            for (int i = 0; i < 10; i++)
                tmpC[i] = 0;
            numc = 0;
            mode++;
            //////////////////////////////////
            while ((tmpchar = fgetc(myFile)) != ',') {
                tmpD[numd] = tmpchar;
                numd++;
            }
            //*acc_param = str2uint(tmpD, numd); //関数でdoubleに変換
            *acc_param = str2double(tmpD, numd);
            if(set_print) printf("%lf, ",*acc_param);
            


            // Serial.print(tmpA);
            if(set_print) printf("%d, ", *acc_param);
            for (int i = 0; i < 10; i++)
                tmpD[i] = 0;
            numd = 0;
            acc_param++;
            //////////////////////////////////
            while (((tmpchar = fgetc(myFile)) != '\r' && tmpchar != ';') &&
                    tmpchar != '/') {
                tmpE[nume] = tmpchar;
                nume++;
            }
            if (tmpchar == ';') {
                file_end = true;
            } else if (tmpchar == '/') { // コメントアウト対応
                while ((tmpchar = fgetc(myFile)) != '\n');
            } else {
                fgetc(myFile); // "\n"を捨てるため
            }
            *dec_param = str2double(tmpE, nume); //関数でdoubleに変換
            path_num++;

            // Serial.print(tmpB);
            if(set_print) printf("%lf\n", *dec_param);
            for (int i = 0; i < 10; i++)
                tmpE[i] = 0;
            nume = 0;
            dec_param++;
        }
        // Serial.println("vel/angle done!");
        // close the file:
        fclose(myFile);
        if(set_print) printf("path num: %d\n", path_num);
    } else {
        // if the file didn't open, print an error:
        // Serial.println("error opening test.txt");
        return -3;
    }

    if ((int)((point_num - 2) / 3) >= (path_num - 1)) {
        return path_num - 1;
    }
    return -4;
}

double mySDclass::str2double(char *str, int num)
{
    double ret = 0.0;
    bool minus = false;
    int m = 0, keta;

    // マイナス符号が付いているかチェック
    if (str[0] == '-') {
        minus = true;
        m++;
    }

    // 何桁あるかを確認
    keta = m;
    while ((str[keta] != '.') && (keta < num)) {
        keta++;
    }
    keta = keta - (m + 1);

    // 整数部を変換
    for (int i = m; i <= (keta + m); i++) {
        if (str[i] >= 48 && str[i] <= 57) {
            ret += (double)(str[i] - 48) * pow(10.0, keta - (i - m));
        }
    }

    // 小数部を変換
    int n = -1;
    for (int i = keta + m + 2; i < num; i++) {
        if (str[i] >= 48 && str[i] <= 57) {
            ret += (double)(str[i] - 48) * pow(10.0, n);
            n--;
        }
    }
    if (minus)
        return -1.0 * ret;
    return ret;
}

int mySDclass::str2uint(char *str, int num)
{
    num--;
    int ret = 0;
    // bool minus = false;

    // 整数部を変換
    for (int i = 0; i <= num; i++) {
        if (str[i] >= 48 && str[i] <= 57) {
            ret += (double)(str[i] - 48) * pow(10.0, num - i);
        }
    }

    return ret;
}