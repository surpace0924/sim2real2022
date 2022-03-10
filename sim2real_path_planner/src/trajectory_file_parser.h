/**
 * @file trajectory_file_parser.h
 * @brief 独自形式の経路CSVファイルをROSのnav_msgs/Path形式に変換
 * @author Ryoga Sato
 * @date 2022/03/09
 **/
#pragma once

#include <iostream>
#include <regex>
#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <stdexcept>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <nav_msgs/Path.h>

#include "./msg_generator.h"

class TrajectoryFileParser
{
private:
    std::string filepath;               // 経路データが保存されているファイルパス
    std::array<double, 2> tolerance;   // 収束許容範囲 [並進, 回転]
    std::array<double, 2> max_vel;     // 経路中の最大速度 [並進, 回転]
    
    std::string getHomeDirName();
    std::vector<std::vector<double>> parserCsvDataVector();

    std::string trajectory_frame_id = "/map";

public:
    TrajectoryFileParser(/* args */);
    ~TrajectoryFileParser();

    void setFilePath(std::string _filepath);
    void setTrajectoryFrameId(std::string _trajectory_frame_id);
    nav_msgs::Path parse();

    std::array<double, 2> getTolerance();
    std::array<double, 2> getMaxVelocity();
};

TrajectoryFileParser::TrajectoryFileParser(/* args */)
{
}

TrajectoryFileParser::~TrajectoryFileParser()
{
}

/**
 * @brief 経路ファイルのファイルパスを指定する
 * @param _filepath: ファイルパス（絶対パス）
 **/
void TrajectoryFileParser::setFilePath(std::string _filepath)
{
    // ホームディレクトリを示す「~」をホームディレクトリ名に変換しておく
    _filepath = std::regex_replace(_filepath, std::regex("~"), getHomeDirName());
    filepath = _filepath;
    // std::cout << "[trajectory_file_parser] get filepath: " << filepath << std::endl;
}

/**
 * @brief 出力経路データ（nav_msgs/Path）のフレームIDを指定する
 * @param _trajectory_frame_id: 出力経路データ（nav_msgs/Path）のフレームID
 **/
void TrajectoryFileParser::setTrajectoryFrameId(std::string _trajectory_frame_id)
{
    // ホームディレクトリを示す「~」をホームディレクトリ名に変換しておく
    trajectory_frame_id = _trajectory_frame_id;
}

/**
 * @brief ホームディレクトリの名前を取得する
 * @return ホームディレクトリの名前
 **/
std::string TrajectoryFileParser::getHomeDirName()
{
    const char *homedir;
    if ((homedir = getenv("HOME")) == NULL) {
       homedir = getpwuid(getuid())->pw_dir;
    }
    std::string homedir_str = homedir;
    return homedir_str;
}

/**
 * @brief CSVに保存されているデータをdouble型の2次元vectorで取得
 * @param _filepath: ファイルパス（絶対パス）
 **/
std::vector<std::vector<double>> TrajectoryFileParser::parserCsvDataVector()
{
    // 区切り文字
    const char delimiter = ',';

    // 戻り値用の2次元double vector
    std::vector<std::vector<double>> data;

    // ファイルを開く
    std::fstream filestream(filepath);

    // ファイルが開けなかった場合は終了する
    if (!filestream.is_open())
    {
        std::cout << "[trajectory_file_parser] can not open file: " << filepath << std::endl;
        return data;
    }

    // 1行ずつファイルを読み込む
    std::string buffer;
    while(std::getline(filestream,buffer))
    {
        // ファイルから読み込んだ１行の文字列を区切り文字で分けてリストに追加する
        std::vector<double> record;              // １行分の文字列のリスト
        std::istringstream streambuffer(buffer); // 文字列ストリーム
        std::string token;                       // １セル分の文字列

        // １セル分の文字列をdoubleに変換してvectorに追加
        while (getline(streambuffer, token, delimiter))
            record.push_back(std::stod(token));

        // １行分の文字列を出力引数のリストに追加する
        data.push_back(record);
    }
    
    return data;
}

/**
 * @brief CSVファイルを読み込み、nav_msgs::Pathを生成する
 * @return nav_msgs/Path形式のデータ
 **/
nav_msgs::Path TrajectoryFileParser::parse()
{
    nav_msgs::Path trajectory_msg;
    trajectory_msg.header = MsgGenerator::toHeader(trajectory_frame_id);
    
    // 展開
    std::vector<std::vector<double>> csv_data = parserCsvDataVector();

    // 展開されたデータをnav_msgs/Pathデータに変換
    for (int row = 0; row < csv_data.size(); ++row)
    {
        if (row == 0)
        {
            // 先頭行は収束条件と最大速度が格納されている
            tolerance[0] = csv_data[row][0];
            tolerance[1] = csv_data[row][1];
            max_vel[0] = csv_data[row][2];
            max_vel[1] = csv_data[row][3];
        }
        else
        {
            // 2行目以降は経路点群データ
            double x = csv_data[row][0];
            double y = csv_data[row][1];
            double theta = csv_data[row][2];

            auto pose_stamped = MsgGenerator::toPoseStamped(trajectory_frame_id, x, y, theta);
            trajectory_msg.poses.push_back(pose_stamped);
        }
    }

    return trajectory_msg;
}

/**
 * @brief 現在読み込まれてる経路データの許容誤差を返す
 * @return 現在読み込まれてる経路データの許容誤差[並進, 回転]
 **/
std::array<double, 2> TrajectoryFileParser::getTolerance()
{
    return tolerance;
}

/**
 * @brief 現在読み込まれてる経路データ中の最大速度を返す
 * @return 現在読み込まれてる経路データ中の最大速度[並進, 回転]
 **/
std::array<double, 2> TrajectoryFileParser::getMaxVelocity()
{
    return max_vel;
}
