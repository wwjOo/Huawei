#include <bits/stdc++.h>
#include <vector>
#include <unordered_set>
#include <chrono>
using namespace std;


#define OPENCV
#define OUT

#ifdef OPENCV
    // #define STEP

    #include <opencv2/opencv.hpp>
    using namespace cv;

    cv::Scalar colors[10] =
    {
        cv::Scalar(0, 0, 255),     // 红色
        cv::Scalar(0, 255, 0),     // 绿色
        cv::Scalar(255, 0, 0),     // 蓝色
        cv::Scalar(0, 255, 255),   // 黄色
        cv::Scalar(255, 0, 255),   // 紫色
        cv::Scalar(255, 255, 0),   // 青色
        cv::Scalar(0, 165, 255),   // 橙色
        cv::Scalar(180, 105, 255), // 粉红色
        cv::Scalar(128, 128, 128), // 灰色
        cv::Scalar(255, 255, 255), // 白色
    };
#endif

// 带颜色的字符串格式输出
#ifdef OUT
#define CLR "\033[0m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define BLUE "\033[1;34m"
#define msgred(x) (cerr << RED << x << CLR)
#define msggreen(x) (cerr << GREEN << x << CLR)
#define msgyellow(x) (cerr << YELLOW << x << CLR)
#define msgblue(x) (cerr << BLUE << x << CLR)
#define msg(x) (cerr << x)
#else
#define msgred(x)
#define msggreen(x)
#define msgyellow(x)
#define msgblue(x)
#define msg(x)
#endif

// 常量
const int N = 200;
const int robot_num = 10;
const int berth_num = 10;
const int boat_num = 5;
const int cargo_time = 1000; //货物消失时间
const double frame_time = 15; //处理一帧的最大时间ms
const int berth_size = 4; //泊位尺寸
// 变量
char ch[N][N];
int money, boat_capacity, id;
vector<deque<vector<int>>> paths(robot_num); // 存储每个机器人的路径
unordered_map<int, int> mappath[N][N]; //全局路径占据信息(每个点<t时刻,robotid经过>集合)
int total_money = 0; //装船的总价值量
uint32_t robot_ctrl_cnt=0; //全局计数器，用于循环控制机器人
int priority_berths_index[berth_num]; //按优先级排序的泊位索引
uint32_t max_transport_time = 0;
uint32_t min_transport_time = UINT32_MAX;

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
class Timer
{
public:
    string title;
    std::chrono::high_resolution_clock::time_point start;
    Timer() : start(std::chrono::high_resolution_clock::now()) {}
    Timer(string title) : title(title),start(std::chrono::high_resolution_clock::now()) {/*msggreen("["<<title<<"]:start time\n");*/}

    void show(string subtitle)
    {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        msggreen("["<<title << "]:"<< subtitle << " Exec T: " << duration.count() / 1000 << " ms\n");
    }
    void show(void)
    {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        msggreen("["<<title << "]: Exec T: " << duration.count() / 1000 << " ms\n");
    }
    double get(void)
    {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        return (double)duration.count() / 1000.0;
    }
    void reset() { start = std::chrono::high_resolution_clock::now(); }
};
struct Cargo
{
    int x, y;           // 坐标
    int value;          // 价值
    int eid;            // 消失帧id
    double priority;    // 优先级, 越高越优先（平均每帧的价值量）
    int belong;         // 所属泊位

    bool operator<(const Cargo &other) const { return priority < other.priority; }

    Cargo() {}
    Cargo(int value) : value(value) {}
    Cargo(int x, int y, int value, int eid) : x(x), y(y), value(value), eid(eid) {}
};
struct Robot // 机器人
{
    int id;             // 机器人ID
    int x, y, tx, ty;   // 当前坐标，目标点坐标
    int goods;          // 是否携带物品(0:无 1:携带)
    int status;         // 状态(0:恢复状态 1:正常运行)

    vector<int> dir;    // 后序路径,用于机器人运动控制
    int target_berth;   // 目标泊位(-1:无效)，0-9表示泊位index
    int init_serve_berth; //首次服务的泊位(-1:无效)，0-9表示泊位index
    Cargo curcargo;     // 携带的货物
    int idle_time;      // 机器人空闲帧统计
    bool isvalid;       // 是否有效（未被封闭）
    int pull_berth;     // 卸货泊位
    uint16_t path_len;  // 路径长度

    Robot() { 
        target_berth = -1; init_serve_berth = -1; 
        idle_time = 0; isvalid = true; 
        }
    Robot(int startX, int startY) : x(startX), y(startY) {}
};
struct Berth // 泊位
{
    int id;             // 泊位ID
    int x, y;           // 左上角坐标
    int transport_time; // 轮船在该泊位运输到虚拟点的时间(time:1~1000帧,即产生价值的时间)
    int loading_speed;  // 装载速度(Velocity:1~5个/帧)
    int curboat;        // 停靠的船只标号(-1表示空闲, 100+i表示等待船i靠岸, 0-9表示船在港口，-10表示被标记为最后停船泊位, -100表示泊位完成了使命)
    int priority;       // 泊位优先级, 越小越优先
    int val;            // 泊位堆集的价值量
    uint16_t buf[N][N]; // 各个点到泊位的距离
    queue<Cargo> stackedcargos; //泊位当前堆积的货物
    priority_queue<Cargo> belongcargos; //属于该泊位的货物集合，按照货物优先级排序
    int robot_num;      //服务该泊位的机器人个数
    double bcs_ratio;    //当前泊位BCS占比

    //其他泊位离当前泊位的路径数 + 索引, 会依据路径进行排序
    vector<pair<int,int>> neighbors;

    void set_priority(void)
    {
        priority = transport_time*2 + boat_capacity/loading_speed;
        msg("id:"<<id<<" priority:" << priority << "\n");
    }

    Berth() 
    { 
        curboat = -1;
        robot_num = 0;

        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++) 
                buf[i][j] = UINT16_MAX;
    }    
};
struct Boat // 轮船
{
    int pos;    // 所在泊位 -1为虚拟点
    int status; // 状态(0:运输中 1:装货或运输完成 2:泊位外等待)
    int num;    // 货物数量
    int val;    // 船舶当前货物总价值
    int final_berth; //确定最后一次停靠(-1：无效， 0-9：已确定)

    Boat() { num = 0; final_berth = -1;}
};
struct node // A*
{
    int x, y;     // 坐标
    int g, h, f;  // 代价
    node *parent; // 父节点
    node(int x, int y, int g, int h, node *parent) : x(x), y(y), g(g), h(h), parent(parent) { f = g + h; }
};

// 变量
Robot robot[robot_num];
Berth berth[berth_num];
Boat boat[boat_num];
auto astarcmp = [](const node *a, const node *b){ return a->f > b->f; }; // 优先队列
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
// 路径规划算法('#'：障碍， '*'：海洋)
double astar_maxt = 0;
int astar_failn = 0;
void dijkstra(char arr[N][N], pair<int,int> start, uint16_t distbuf[N][N]) 
{
    typedef pair<int, pair<int, int>> pipii; //first: distance, second: coordinates (x, y)
    queue<pipii> q;
    q.push(make_pair(0, start));
    distbuf[start.first][start.second] = 0;

    // 开始BFS
    vector<pair<int, int>> dirs = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    while (!q.empty()) 
    {
        pipii cur = q.front(); q.pop();

        // 遍历当前节点的相邻节点
        for (auto dir : dirs) 
        {
            int x = cur.second.first + dir.first;
            int y = cur.second.second + dir.second;
            if (x >= 0 && x < N && y >= 0 && y < N && (arr[x][y] == '.' || arr[x][y] == 'B' || arr[x][y] == 'A') && distbuf[x][y] == UINT16_MAX) 
            {
                distbuf[x][y] = distbuf[cur.second.first][cur.second.second] + 1;
                q.push({distbuf[x][y],{x, y}});
            }
        }
    }
}
int Astar(char arr[N][N], int zhen, int startX, int startY, int &endX, int &endY, int i, int action, deque<vector<int>> &path, Timer &t) 
{
    Timer astarT;
    int res = 0; //-1:快到帧时间了 0:失败 1:成功
    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};

    vector<vector<uint16_t>> gValue(N, vector<uint16_t>(N, UINT16_MAX)); // 存储从起点到每个节点的实际代价值
    priority_queue<node *, vector<node *>, decltype(astarcmp)> pq(astarcmp);
    unordered_set<node *> item_to_delete; // 存储待删除的节点，避免内存泄露

    node *first = new node(startX, startY, 0, abs(startX - endX) + abs(startY - endY), nullptr);
    item_to_delete.insert(first);
    pq.push(first);

    while (!pq.empty())
    {
        #ifndef STEP
        if (t.get() >= frame_time - 1) 
        {
            res = -1;
            break; // 即将超过帧时间，退出
        }
        #endif  

        node *cur = pq.top(); pq.pop();

        //取货是以点为目标，卸货是以区块为目标
        if ((action == 1 && cur->x >= endX && cur->y >= endY && cur->x < endX+berth_size && cur->y < endY+berth_size) || //卖货
            (action == 0 && cur->x == endX && cur->y == endY)) //取货
        {
            endX = cur->x;
            endY = cur->y;
            robot[i].dir.clear();
            path.clear();
            
            while (cur->parent) // 不包含当前机器人位置
            {
                //添加path(不包含当前机器人位置，逆序)
                path.push_back(vector<int>({cur->x, cur->y}));

                //添加dir(逆序)
                int px = cur->parent->x, x = cur->x;
                int py = cur->parent->y, y = cur->y;
                if (px == x)
                {
                    if (py < y) robot[i].dir.push_back(0); // 向右
                    else robot[i].dir.push_back(1); // 向左
                }
                else
                {
                    if (px > x) robot[i].dir.push_back(2); // 向上
                    else robot[i].dir.push_back(3); // 向下
                }

                //下一个节点
                cur = cur->parent;
            }

            res = 1;
            break;
        }

        // 遍历当前点的四个方向
        for (int i = 0; i < 4; ++i)
        {
            int newX = cur->x + dx[i];
            int newY = cur->y + dy[i];

            // 空地 or 泊位
            if (newX >= 0 && newX < N && newY >= 0 && newY < N && gValue[newX][newY] == UINT16_MAX && (arr[newX][newY] == '.' || arr[newX][newY] == 'B' || arr[newX][newY] == 'B'))
            {
                int newG = cur->g + 1;
                int newH = abs(newX - endX) + abs(newY - endY);

                gValue[newX][newY] = newG;
                node *newnode = new node(newX, newY, newG, newH, cur);
                pq.push(newnode);
                item_to_delete.insert(newnode);
            }
        }
    }
    // 绘制
    #ifdef STEP
    if(t.get() > 5)
    {
        // 画出地图
        int rows = 200, cols = 200;
        Mat grayImage(rows, cols, CV_8UC1);
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
                grayImage.at<uchar>(i, j) = static_cast<uchar>(gValue[i][j] == UINT16_MAX ? 0 : 255);

        // 改为彩图
        cv::Mat colorImage;
        cv::cvtColor(grayImage, colorImage, cv::COLOR_GRAY2BGR);
        cv::circle(colorImage, Point(endY, endX), 2, cv::Scalar(255, 0, 255), -1);

        // 放大图像
        cv::resize(colorImage, colorImage, cv::Size(600, 600));
        imshow("Image", colorImage);
        waitKey(0);
    }
    #endif

    if(astarT.get() > astar_maxt) astar_maxt = astarT.get();

    // 删除，避免内存泄露
    for (auto i = item_to_delete.begin(); i != item_to_delete.end(); ++i) delete (*i);
    return res;
}


// 帧显示
#ifdef OPENCV
void display(vector<deque<vector<int>>> &paths, int waittime)
{
    // 画出地图
    int rows = 200, cols = 200;
    Mat grayImage(rows, cols, CV_8UC1);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            grayImage.at<uchar>(i, j) = static_cast<uchar>(ch[i][j]);

    // 改为彩图
    cv::Mat colorImage;
    cv::cvtColor(grayImage, colorImage, cv::COLOR_GRAY2BGR);

    // 画出机器人位置 + 路径
    for (int i = 0; i < robot_num; i++)
    {
        cv::circle(colorImage, cv::Point(robot[i].y, robot[i].x), 1, cv::Scalar(255, 0, 255), -1);
        cv::circle(colorImage, cv::Point(robot[i].ty, robot[i].tx), 1, cv::Scalar(127, 127, 255), -1);
        cv::putText(colorImage, std::to_string(i), cv::Point(robot[i].y, robot[i].x), cv::FONT_HERSHEY_SIMPLEX, 0.3, colors[i], 1);
        for (auto &point : paths[i])
            cv::circle(colorImage, cv::Point(point[1], point[0]), 0.1, colors[i], -1);
    }    
    // 画出泊位
    for(int i=0; i<berth_num; i++)
    {       
        cv::putText(colorImage, std::to_string(i), cv::Point(berth[i].y, berth[i].x), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(127, 127, 127), 1);
        cv::rectangle(colorImage, cv::Point(berth[i].y, berth[i].x), cv::Point(berth[i].y+berth_size-1, berth[i].x+berth_size-1), cv::Scalar(127, 127, 127), 1, LINE_8, 0);
    }   
    
    //zhen
    cv::putText(colorImage, std::to_string(id), cv::Point(0, 10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 0.1);

    // 放大图像
    cv::resize(colorImage, colorImage, cv::Size(700, 700));
    imshow("Image", colorImage);
    waitKey(waittime);
}
#endif

//==================================================================================================================//
//==================================================================================================================//
//==================================================================================================================//
//设置货物归属+优先级
bool set_cargo_belong_berth(Cargo c)
{
    int belong_berth = -1;
    int minpathlen = INT_MAX;
    for (int i = 0; i < berth_num; i++)
    {
        int curpathlen = berth[i].buf[c.x][c.y];
        if(curpathlen < minpathlen)
        {
            minpathlen = curpathlen;
            belong_berth = i;
        }
    }
    if(belong_berth == -1) return false;

    //更改货物消失时间(减去从泊位到机器人的帧数)
    c.eid -= minpathlen+50;
    // 确定优先级(单位帧的价值量)
    c.priority = (double)c.value / (double)minpathlen;
    // 添加所属泊位
    c.belong = belong_berth;
    // 添加到所属泊位货物容器
    berth[belong_berth].belongcargos.push(c);

    return true;
}
//获取邻居的距离优先对列
void get_berth_neighbors(Berth &b)
{
    for(int i=0; i<berth_num; i++)
    {
        if(b.id == berth[i].id) continue;

        int dist = b.buf[berth[i].x][berth[i].y];
        if( dist != UINT16_MAX ) 
            b.neighbors.push_back(make_pair(dist, i));
    }
    if(b.neighbors.size() != 0)
        sort(b.neighbors.begin(),b.neighbors.end(),[](pair<int,int> &aa, pair<int,int> &bb){return aa.first < bb.first;});
}

//******机器人货物选取策略*************
Cargo Strategy_Robot_Cargo_Choose(int zhen, Robot &r)
{
    Cargo target(-1);

    //自己的泊位上找
    while ( !berth[r.target_berth].belongcargos.empty() )
    {
        Cargo front = berth[r.target_berth].belongcargos.top();
        berth[r.target_berth].belongcargos.pop();

        //有效 - 选定
        if(front.eid > id) 
        {
            target = front;
            break;
        }
    }

    //自己泊位上没有多余所属货物
    if(target.value == -1 && r.idle_time > 0)
    {
        //在邻近2个泊位上找
        int neighborid = -1;
        int num = berth[r.target_berth].neighbors.size() > 4 ? 4 : berth[r.target_berth].neighbors.size();
        for(int k=0; k<num; k++)
        {
            //选定邻居泊位，退出选择
            if(berth[berth[r.target_berth].neighbors[k].second].belongcargos.size() >= 2) 
            {
                neighborid = berth[r.target_berth].neighbors[k].second;
                break; 
            }
        }

        //有效选择
        if(neighborid != -1)
        {
            //取出有效货物
            while( !berth[neighborid].belongcargos.empty() )
            {
                Cargo front = berth[neighborid].belongcargos.top();
                berth[neighborid].belongcargos.pop();

                //有效 - 选定
                if(front.eid > id) 
                {
                    target = front;
                    break;
                }
            }
        }
    }

    return target;
}
//******机器人泊位选取策略*************
void Strategy_Robot_Berth_Choose(int zhen, Robot &r)
{
    static bool isinit = false;

    //初始化
    if(isinit == false)
    {
        //到未分配机器人泊位的距离打包<距离 泊位>
        vector<pair<int, int>> pathslen;
        for(int i=0; i<berth_num; i++)
            if(berth[i].robot_num == 0)
                pathslen.push_back({berth[i].buf[r.x][r.y], i});
           
        //存在空闲泊位
        if(pathslen.size() != 0)
        {
            //对距离进行排序
            sort(pathslen.begin(),pathslen.end(),[](const pair<int, int> &a, const pair<int, int> &b){return a.first < b.first;});
            
            //选取最近一个空闲泊位
            r.target_berth = pathslen[0].second;
            r.init_serve_berth = r.target_berth;
            berth[pathslen[0].second].robot_num ++;
            r.idle_time = 0;
        }
        else
        {
            isinit = true;
        }
    }
    //3.22add
    //泊位完成使命后的机器人调度
    else if(r.target_berth != -1 && berth[r.target_berth].curboat == -100)
    {
        //寻找合适泊位：按到邻近泊位的距离从小到大筛选 
        for(int k=0; k<berth[r.target_berth].neighbors.size(); k++)
        {
            if(berth[berth[r.target_berth].neighbors[k].second].curboat == -100) continue;
            if(berth[berth[r.target_berth].neighbors[k].second].curboat >= 0 && berth[berth[r.target_berth].neighbors[k].second].curboat < berth_num && berth[berth[r.target_berth].neighbors[k].second].robot_num >= 2) continue;

            if( berth[berth[r.target_berth].neighbors[k].second].curboat == -10 || 
                (berth[berth[r.target_berth].neighbors[k].second].curboat >= 100 && boat[berth[berth[r.target_berth].neighbors[k].second].curboat-100].final_berth != -1) ||
                (berth[berth[r.target_berth].neighbors[k].second].curboat >=0 && berth[berth[r.target_berth].neighbors[k].second].curboat < boat_num && boat[berth[berth[r.target_berth].neighbors[k].second].curboat].final_berth != -1))
            {
                berth[r.target_berth].robot_num --;
                r.target_berth = berth[r.target_berth].neighbors[k].second;
                berth[r.target_berth].robot_num ++;
                r.idle_time = 0;

                break;
            }
        }
    }
    // 长期分配策略
    else
    {
        // 前提： 机器人空闲帧大于阈值 + 泊位没有所属货物 + 泊位没有船
        if(r.idle_time > 100 && berth[r.target_berth].belongcargos.size() == 0 && berth[r.target_berth].curboat == -1)
        {
            int num = berth[r.target_berth].neighbors.size() > 3 ? 3 : berth[r.target_berth].neighbors.size();
            for(int k=0; k<num; k++) //选择邻近的两个泊位
            {
                if(berth[berth[r.target_berth].neighbors[k].second].robot_num >= 3) continue; //每个泊位限制最多3个机器人进行服务

                if(berth[berth[r.target_berth].neighbors[k].second].belongcargos.size() > 4)
                {
                    berth[r.target_berth].robot_num --;
                    r.target_berth = berth[r.target_berth].neighbors[k].second;
                    berth[r.target_berth].robot_num ++;
                    r.idle_time = 0;

                    break;
                }
            }
        }
    }
}
//******船从虚拟点返回的泊位选取策略****************
int Strategy_Boat_VtoB_Choose(int zhen, Boat &b) //0～9泊位索引id
{
    int berthid = -1;
    int flag = 0;

    //泊位存在货物时 ：按泊位堆积的每帧最大价值来
    double max_value_per_fr = 0;
    for (int k = 0; k < berth_num; k++)
    {
        //泊位上没有货物
        if (berth[k].stackedcargos.size() == 0 || berth[k].robot_num == 0) continue;

        double per_val = (double)(berth[k].val)/(double)(berth[k].transport_time);
        if (berth[k].curboat == -1 && (berthid == -1 || per_val > max_value_per_fr))
        {
            berthid = k;
            max_value_per_fr = per_val;
        }

        if(berth[k].stackedcargos.size() > boat_capacity/2) flag = 1;
    }

    //第二策略
    if(flag == 0)
    {
        for(int i=0; i<berth_num; i++)
        {
            if(berth[i].robot_num == 0) continue;

            if(berth[i].curboat == -1 && berth[i].stackedcargos.size() > 0 && (berthid == -1 || berth[i].priority < berth[berthid].priority))
                berthid = i;
        }
    }

    //所有泊位都没有货物时 ：按优先级来
    if(berthid == -1)
    {
        for(int i=0; i<berth_num; i++)
        {
            if(berth[i].curboat == -1 && (berthid == -1 || berth[i].priority < berth[berthid].priority))
                berthid = i;
        }
    }

    //3.22add
    //最后停船时，停靠预定船位
    if(b.final_berth != -1)
        berthid = b.final_berth;

    return berthid;
}
//******船卖货时机选取策略***************
bool Strategy_Meet_SaleCond(int zhen, Boat &b) //true:可以卖货
{
    if(b.pos == -1 || b.pos>=100) return false; //船没在泊位，放弃

    //3.22add
    //倒数第二次策略
    bool go_flag = false;
    if(berth[b.pos].stackedcargos.size() == 0) //当前泊位堆积的货物必须要先装完船
    {
        //确定当前船舶的最后一次停靠泊位
        for(int i=0; i<berth[b.pos].neighbors.size(); i++)
        {
            Berth iberth = berth[berth[b.pos].neighbors[i].second];
            int needtime =  berth[b.pos].transport_time + //从当前泊位卖货时间
                            (double)iberth.stackedcargos.size()/(double)(iberth.loading_speed) + //最后泊位装货时间
                            2*iberth.transport_time; //来回最后泊位的时间

            //满足所需时间 且 泊位空闲
            if(14999 - id -10 <= needtime && needtime <= 14999 - id && berth[i].curboat == -1)
            {
                berth[i].curboat = -10; //预定泊位i
                berth[b.pos].curboat = -100; //当前泊位标记完成使命
                b.final_berth = i;
                go_flag = true;
                break;
            }
        }
    }
    if(go_flag == true) return true;

    //船要有货 && 比赛结束阶段
    //船装满
    return b.num > 0 && (14999 - id) < berth[b.pos].transport_time ||
           b.num >= boat_capacity;
}
//******船在泊位之间的调度策略****************
int Strategy_Boat_BtoB_Choose(int zhen, Boat &b) //-1:没有合适泊位，0～9泊位索引id
{
    int suitid = -1;

    //泊位累积货物都已装船(必须满足)
    //船当前余量 > 20个 || 泊位所属货物为0 且 泊位累积货物为0
    if( (berth[b.pos].robot_num == 0 && berth[b.pos].stackedcargos.size() == 0) || (berth[b.pos].stackedcargos.size() == 0 && boat_capacity - b.num > 20))
    {
        int idle = boat_capacity - b.num;
        for(int i=0; i<berth_num; i++)
        {
            //一级条件(必须满足)：泊位空闲 + 容许最后一次能够卖出
            if( berth[i].curboat == -1 && berth[i].transport_time + 500 < 14900 - id)
            {
                //二级条件:其他泊位刚好能装满船(5个货物可以在转移时转) // 当前泊位所属货物没有了
                if(berth[i].stackedcargos.size() >= idle - 5 || (berth[i].stackedcargos.size() > 10 && berth[b.pos].belongcargos.size() == 0))
                { 
                    // //3.22add
                    // //增加判断：调度后是否会影响最后的卖货
                    int t_berth = i; //当前遍历的目标泊位
                    bool ispermission = true; //允许标记
                    for(int k=0; k<berth_num; k++)
                    {
                        //跳过当前泊位和目标泊位
                        if(k == t_berth || berth[k].curboat != -1) continue;

                        //所需时间片= BtoB时间 + 在t_berth装船时间 + t_berth运输时间 + 往返当前遍历泊位时间 + 在当前遍历泊位装货时间

                        int needtime =  500 + (double)(boat_capacity - b.num)/(double)berth[t_berth].loading_speed +
                                        berth[t_berth].transport_time +
                                        2*berth[k].transport_time +
                                        (double)berth[k].stackedcargos.size()/(double)(berth[k].loading_speed);
            
                        //保守条件：只要有一个不满足则不允许调度
                        if(needtime > 14900 - id)
                        {
                            ispermission = false;
                            break;
                        };
                    }
                    if(ispermission == false) continue;

                    if(suitid == -1 || berth[i].priority < berth[suitid].priority)
                        suitid = i;
                }
            }
        }
    }

    if(b.pos == suitid) suitid = -1;
    
    return suitid;
}

//初始化
void Init()
{
    // 地图数据
    for (int i = 0; i < N; i++)
        scanf("%s", ch[i]);

    // 注意：判题器是按固定顺序下发泊位信息的，不能对泊位数组重排序，后序的泊位停靠使用的该id
    // 10个泊位数据 左上角坐标 + 该泊位运输到虚拟点时间(time:1~1000帧,即产生价值的时间) + 装载速度(Velocity:1~5个/帧)
    for (int i = 0; i < berth_num; i++)
    {
        int id;
        scanf("%d", &id);
        berth[id].id = id;
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
        msg("berth info" << id << " :" << berth[id].x << "," << berth[id].y << "\n");

        if(berth[id].transport_time > max_transport_time) max_transport_time = berth[id].transport_time;
        if(berth[id].transport_time < min_transport_time) min_transport_time = berth[id].transport_time;
    }

    // 船的容积
    scanf("%d", &boat_capacity);
    msg("boat_capacity:"<<boat_capacity<<"\n");
    
    // 设置泊位优先级 + 地图每个点到泊位的距离 + 获得与其他泊位的相对距离信息
    vector<pair<int,int>> buf(berth_num);
    for(int i=0; i<berth_num; i++)
    {
        berth[i].set_priority();
        buf[i] = make_pair(i,berth[i].priority);

        Timer t("dijkstra");
        dijkstra(ch,make_pair(berth[i].x,berth[i].y),berth[i].buf);
        t.show();

        get_berth_neighbors(berth[i]);
        for(auto &i : berth[i].neighbors)
            msg("["<<i.first<<","<<i.second<<"] ");
        msg("\n");
    }

    //更新优先级(0-9)
    sort(buf.begin(),buf.end(),[](pair<int,int> &a, pair<int,int> &b){return a.second < b.second;});
    for(int i=0; i<berth_num; i++)
    {
        berth[buf[i].first].priority = i;
        priority_berths_index[i] = buf[i].first; //按优先级保存索引
    }

    //标记机器人ID
    for(int i=0; i<robot_num; i++)
        robot[i].id = i;

    // finish
    char okk[100];
    scanf("%s", okk);
    printf("OK\n");
    fflush(stdout);
}
//判题输入
int Input()
{
    static int robot_isvalid_check_finish = 0;

    // 当前帧序号 + 当前金钱数
    scanf("%d%d", &id, &money);
    msgblue("--------"<< "ID:" << id << "---" << "Money:" << money<<"---my:"<< total_money << "--------\n");

    // 新增货物的数量(0~10)，以及这些货物的信息(货物的坐标 + 货物的金额:正整数<=1000)
    int num;
    scanf("%d", &num);
    for (int i = 1; i <= num; i++)
    {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);

        //设置归属
        set_cargo_belong_berth(Cargo(x, y, val, id + cargo_time));
    }

    //更新一次bcs占比
    {
        for(int i=0; i<berth_num; i++)
            berth[i].bcs_ratio = (double)berth[i].belongcargos.size()/(double)boat_capacity;
    }

    // 10个机器人的状态 是否携带物品(0:无 1:携带) + 坐标(x,y) + 状态(0:恢复状态 1:正常运行)
    msg("HaveGood robot ID:");
    for (int i = 0; i < robot_num; i++)
    {
        scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y, &robot[i].status);
        if (robot[i].goods) msg(i << "  ");
    }
    msg("\n");

    msg("Closed robot ID:");
    for(int k=0; k<robot_num; k++)
    {
        if(!robot[k].isvalid) msg(k);
    }
    msg("\n");

    //判断机器人是否被封闭
    if(robot_isvalid_check_finish == 0)
    {
        for(int k=0; k<robot_num; k++)
        {
            //每个机器人进行判断
            int isvalid = false;
            for(int b=0; b<berth_num; b++)
            {
                if(berth[b].buf[robot[k].x][robot[k].y] != UINT16_MAX)
                {
                    isvalid = true;
                    break;
                }
            }
            robot[k].isvalid = isvalid;
        }

        robot_isvalid_check_finish = 1;
    }

    // 5艘船的状态(0:运输中 1:装货或运输完成 2:泊位外等待) + 目标泊位(-1代表虚拟点 0-9)
    for (int i = 0; i < boat_num; i++)
    {
        // 当状态为1且pos为-1时表示抵达虚拟点
        // 当状态为1且pos不为1时表示到达港口
        scanf("%d%d\n", &boat[i].status, &boat[i].pos);

        // 船已到达指定泊位
        if (boat[i].status == 1 && boat[i].pos != -1 && berth[boat[i].pos].curboat == 100 + i)
        {
            berth[boat[i].pos].curboat = i;
        }

        //船在泊位后进行装货
        if(boat[i].status == 1 && boat[i].pos != -1 && berth[boat[i].pos].curboat == i)
        {
            for(int k=0; k<berth[boat[i].pos].loading_speed; k++)
            {
                if(berth[boat[i].pos].stackedcargos.empty()) break;

                //装船
                Cargo c = berth[boat[i].pos].stackedcargos.front();
                berth[boat[i].pos].stackedcargos.pop();

                //更新信息
                boat[i].num++;
                boat[i].val += c.value;
                berth[boat[i].pos].val -= c.value;
            }
        }
    }
    
    // 打印泊位信息
    for (int i = 0; i < berth_num; i++)
    {
        //泊位基础信息
        msg("berth[" << i << "]"<<" prior:");
        if(berth[i].priority < 5) {msggreen(berth[i].priority);msggreen(berth[i].priority);msggreen(berth[i].priority);}
        else msg(berth[i].priority);
        msg("\tN/fr:"<< berth[i].loading_speed << "\tTtoV:" << berth[i].transport_time<< "\tcurboat:"<<berth[i].curboat << "\tBCs:" << 
            berth[i].belongcargos.size()<<"\t("<<setprecision(2)<<fixed<< (berth[i].bcs_ratio) <<")" << 
            "  robots:"<<berth[i].robot_num <<"   Num:" << berth[i].stackedcargos.size() << "("<<berth[i].val<<")");
       
        //泊位停船信息
        if (0 <= berth[i].curboat && berth[i].curboat < boat_num)
            msgblue("\t<-[" << berth[i].curboat << "]  Num:" << boat[berth[i].curboat].num<<"("<<boat[berth[i].curboat].val<<")" << " capacity:" << boat_capacity);
        if (berth[i].curboat >= 100)
            msgblue("\t\t<...");

        //船是否在虚拟点
        if(boat[i].pos == -1 && boat[i].status == 1)
            msgyellow("\t\t\t |**|");
        msg("\n");
    }

    // finish
    char okk[100];
    scanf("%s", okk);
    return id;
}


//抓取货物
int GetCargo(int zhen, Timer &t)
{
    int success = 0;

    for(int r=0; r<robot_num; r++)
    {
        int i = robot_ctrl_cnt % 10; //当前遍历的机器人索引

        if( !robot[i].isvalid )
        {
            robot_ctrl_cnt++;
            continue;
        }

        //快到帧时间了
        if(t.get() >= frame_time-0.5) break;

        // 规划条件：正常运行 && 无货物 && 无目标点
        if(robot[i].status == 1 && robot[i].goods == 0 && robot[i].tx == 0)
        {
            //分配泊位
            Strategy_Robot_Berth_Choose(zhen, robot[i]);

            //分配货物
            Cargo target = Strategy_Robot_Cargo_Choose(zhen, robot[i]);

            //成功分配货物
            if(target.value != -1)
            {
                // 规划路径
                int res = Astar(ch, zhen, robot[i].x,robot[i].y, target.x, target.y, i, 0, paths[i], t);
                if( res == 1 && 
                    (robot[i].dir.size() + id >= target.eid || 
                     robot[i].dir.size() >= berth[robot[i].target_berth].buf[target.x][target.y] + 50)) 
                    res = -1;

                if(res == 1)
                {
                    //成功
                    robot[i].tx = target.x;
                    robot[i].ty = target.y;
                    robot[i].curcargo = target; //机器人当前搬运的货物
                    success++; //记录本帧成功规划的机器人
                }
                else if(res == -1) //超时
                {
                    //失败，统计失败次数
                    astar_failn++;

                    //如果是超时，则将当前取出的货物放回去
                    if(res == -1) berth[target.belong].belongcargos.push(target);
                }
            }
            else //未成功分配，机器人将空闲
            {
                robot[i].idle_time++;
            }
        }
        robot_ctrl_cnt++;
    }
    
    return success;
}
//卸载货物
int PullCargo(int zhen, Timer &t)
{
    int success = 0;

    for(int k=0; k<robot_num; k++)
    {
        int i = robot_ctrl_cnt % robot_num; //获取当前处理的机器人索引

        if( !robot[i].isvalid )
        {
            robot_ctrl_cnt++;
            continue;
        }

        //快到帧时间了
        if(t.get() >= frame_time-0.5) break;
        
        // 规划条件：正常运行 && 携带货物 && 无目标点
        if (robot[i].status == 1 && robot[i].goods == 1 && robot[i].tx == 0)
        {
            //--------------------------------------------------------------------
            int best_berth = robot[i].target_berth;
            if(berth[robot[i].target_berth].curboat == -1)
            {
                for(int k=0; k<berth_num; k++)
                {
                    if(berth[k].buf[robot[i].x][robot[i].y] < berth[best_berth].buf[robot[i].x][robot[i].y]) 
                    {
                        best_berth = k;
                    }
                }
            }

            //实际的目标点在Astar中更新：泊位是一个区域，只要搜索到区域则成功，并更改终点坐标
            int tx = berth[best_berth].x;
            int ty = berth[best_berth].y;
            robot[i].pull_berth = best_berth;
            
            int res = Astar(ch, zhen, robot[i].x, robot[i].y, tx, ty, i, 1, paths[i], t);
            if( res == 1 && (robot[i].dir.size() >= berth[best_berth].buf[robot[i].x][robot[i].y] + 25)) 
                res = -1;

            if (res == 1) //成功
            {
                robot[i].tx = tx;
                robot[i].ty = ty;
                success++;
            }
            else
            {
                astar_failn++; //统计失败次数
            }
        }
        robot_ctrl_cnt++;
    }
    
    return success;
}

//控制船
void boatcmd(int zhen)
{
    for (int i = 0; i < boat_num; i++)
    {
        // 虚拟点返回泊位
        if (boat[i].pos == -1 && boat[i].status == 1)
        {
            // 船的货物数量，价值归零
            boat[i].num = 0;
            boat[i].val = 0;

            //选择合适泊位
            int berthindex = Strategy_Boat_VtoB_Choose(zhen, boat[i]);

            //标记船正处于来的路上
            berth[berthindex].curboat = 100 + i;

            printf("ship %d %d\n", i, berthindex);
        }
        
        // 泊位卖货至虚拟点
        if ( Strategy_Meet_SaleCond(zhen, boat[i]) == true )
        {
            if((boat[i].final_berth != -1 && boat[i].pos == boat[i].final_berth) || 
                berth[boat[i].pos].transport_time + max_transport_time > 15000 - id)
                berth[boat[i].pos].curboat = -100;

            // 清除泊位船只记录
            if(berth[boat[i].pos].curboat != -100) berth[boat[i].pos].curboat = -1; 

            // 更新装船的总价值
            total_money += boat[i].val; 

            printf("go %d\n", i);
        }

        //泊位间调度：前提：在泊位且处于装货状态
        if(0<=boat[i].pos && boat[i].pos<berth_num && boat[i].status == 1)
        {
            //选择泊位
            int berthindex = Strategy_Boat_BtoB_Choose(zhen, boat[i]);

            //有效泊位
            if(berthindex != -1)
            {
                berth[boat[i].pos].curboat = -1; // 清除当前泊位船只
                berth[berthindex].curboat = 100 + i; //标记船正赶往新泊位

                printf("ship %d %d\n", i, berthindex);
            }
        }
    }
}

//机器人控制
bool isdirfree(int curx, int cury, int dir)
{//检测dir方向是否没有障碍
    if(dir == 0 && cury+1 < N) return ch[curx][cury+1] == '.' || ch[curx][cury+1] == 'B' || ch[curx][cury+1] == 'A';
    if(dir == 1 && cury-1 >= 0) return ch[curx][cury-1] == '.' || ch[curx][cury-1] == 'B' || ch[curx][cury+1] == 'A';
    if(dir == 2 && curx-1 >= 0) return ch[curx-1][cury] == '.' || ch[curx-1][cury] == 'B' || ch[curx][cury+1] == 'A';
    if(dir == 3 && curx+1 < N) return ch[curx+1][cury] == '.' || ch[curx+1][cury] == 'B' || ch[curx][cury+1] == 'A';

    return false;
}
bool isinberth(Berth &b, int x, int y)
{//检测机器人是否到达泊位
    if(x < b.x || y < b.y || x >= b.x + berth_size || y >= b.y+berth_size) return false;
    
    return true;
}
void robotcmd(int zhen)
{
    int dx[4] = {0, 0, -1, 1};
    int dy[4] = {1, -1, 0, 0};
    for (int i = 0; i < robot_num; i++)
    {
        // 取货控制
        if (robot[i].goods == 0 && robot[i].x == robot[i].tx && robot[i].y == robot[i].ty)
        {
            printf("get %d\n", i);
            robot[i].tx = 0;
            robot[i].ty = 0;
        }
        // 移动控制
        if (robot[i].dir.size() != 0)
        {
            //下一步会与其他机器人发生碰撞
            if(ch[paths[i].back()[0]][paths[i].back()[1]] >=0 && ch[paths[i].back()[0]][paths[i].back()[1]] < robot_num)
            {              
                int curx = robot[i].x;
                int cury = robot[i].y;

                //向四个方向找到空闲位置
                int changed = 0;
                int dir = (robot[i].dir.back() >=2 ? 0 : 2);
                for(int k=0; k<4; k++)
                {     
                    if( isdirfree(curx,cury,(dir+k)%4) )
                    {
                        dir = (dir+k)%4;

                        //下一步
                        if(dir == 0) robot[i].dir.push_back(1);
                        if(dir == 1) robot[i].dir.push_back(0);
                        if(dir == 2) robot[i].dir.push_back(3);
                        if(dir == 3) robot[i].dir.push_back(2);
                        paths[i].push_back({curx,cury});

                        //当前步
                        robot[i].dir.push_back(dir);
                        paths[i].push_back({curx+dx[dir],cury+dy[dir]});

                        changed = 1;
                        break;
                    }
                }

                //没地方走了
                if( changed == 0 )
                {
                    //当前步
                    robot[i].dir.push_back(-1);
                    paths[i].push_back({robot[i].x,robot[i].y});
                }
            }

            printf("move %d %d\n", i, robot[i].dir.back());

            ch[robot[i].x][robot[i].y] = '.'; // 解除机器人当前位置障碍
            ch[paths[i].back()[0]][paths[i].back()[1]] = i; // 下一位置设为障碍

            robot[i].dir.pop_back();
            paths[i].pop_back();
        }
        else
        {  
            int dir = -1;
            int curx = robot[i].x;
            int cury = robot[i].y;

            for(int j=0; j<4; j++)
            {
                //四周是否与其他机器人发生碰撞
                if(ch[curx+dx[j]][cury+dy[j]] >=0 && ch[curx+dx[j]][cury+dy[j]] < robot_num)
                {                
                    //向四个方向找到空闲位置
                    for(int k=0; k<4; k++)
                    {
                        if( isdirfree(curx,cury,k) )
                        {
                            dir = k;
                            break;
                        }
                    }
                }
                if(dir != -1) 
                {
                    printf("move %d %d\n", i, dir);
                    ch[robot[i].x][robot[i].y] = '.'; // 解除机器人当前位置障碍
                    ch[robot[i].x + dx[dir]][robot[i].y + dy[dir]] = i; // 下一位置设为障碍
                    break;
                }   
            } 
        }

        // 卸货控制
        // if (robot[i].goods == 1 && robot[i].x == robot[i].tx && robot[i].y == robot[i].ty)
        if(robot[i].goods == 1 && isinberth(berth[robot[i].pull_berth], robot[i].x, robot[i].y))
        {
            robot[i].dir.clear();
            paths[i].clear();

            printf("pull %d\n", i);
            robot[i].tx = 0;
            robot[i].ty = 0;

            //更新泊位积压的货物信息
            berth[robot[i].pull_berth].stackedcargos.push(robot[i].curcargo);
            berth[robot[i].pull_berth].val += robot[i].curcargo.value;
        }
    }
}

void ctrl(int zhen, Timer &t)
{
    //---------------------------------------------------------------------------------------
    // 取货路径规划
    //---------------------------------------------------------------------------------------
    int res = GetCargo(zhen,t);
    msg("-------"<<res<<" robot get cargo Astar success-------\n");
    //---------------------------------------------------------------------------------------
    // 卸货路径规划
    //---------------------------------------------------------------------------------------
    res = PullCargo(zhen,t);
    msg("-------"<<res<<" robot pull cargo Astar success-------\n");
    //---------------------------------------------------------------------------------------
    // 控制 - 货船
    //---------------------------------------------------------------------------------------
    boatcmd(zhen);
    //---------------------------------------------------------------------------------------
    // 控制 - 机器人
    //---------------------------------------------------------------------------------------
    robotcmd(zhen);
}

int main(int argc, char **argv)
{
    double averaget = 0; // 平均运行时间
    int overtime = 0;   // 超时时间
    
    Init();

    for (int zhen = 1; zhen <= 15000; zhen++)
    {
        msgblue("===============zhen:" << zhen << "===============\n");
        Timer t("zhen");

        //输入状态
        int id = Input();

        //输出控制
        ctrl(zhen, t);

        #ifdef OPENCV
            display(paths, 1);
            while(t.get() < 12);
        #endif
        
        msgyellow("astar max t:"<<astar_maxt<<endl);
        msgyellow("astar failed t:"<<astar_failn<<endl);

        puts("OK");
        fflush(stdout);
        t.show("all");


        if (t.get() >= frame_time) ++overtime;
        averaget += t.get();
        if (id == 15000)
        {
            averaget /= 15000;
            msg("=======finish:" << id << "-" << zhen << "  avg:" << averaget << "  over:" << overtime << "=======" << endl);
            msg("=======finish:" << id << "-" << zhen << "  avg:" << averaget << "  over:" << overtime << "=======" << endl);
            msg("=======finish:" << id << "-" << zhen << "  avg:" << averaget << "  over:" << overtime << "=======" << endl);
            cerr <<"=======finish:" << id << "-" << zhen << "  avg:" << averaget << "  over:" << overtime << "=======" << endl;
            #ifdef OPENCV
                cv::destroyAllWindows();
            #endif
            return 0;
        }
    }

    return 0;
}
