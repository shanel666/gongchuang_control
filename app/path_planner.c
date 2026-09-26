#include "path_planner.h"
#include "path_config.h"


/* ----------------------------------------------------------------------------
 * 5x5 网格全部 25 个交点坐标（左下角为原点，与你原始坐标一致）
 * 索引 idx = 行*5 + 列
 *   行 r (y): 0=2250, 1=1725, 2=1200, 3=625, 4=150
 *   列 c (x): 0=150,  1=675,  2=1200, 3=1725, 4=2250
 *
 *   索引  坐标          索引  坐标
 *    0   (150,2250)     1   (675,2250)
 *    2   (1200,2250)    3   (1725,2250)
 *    4   (2250,2250)
 *    5   (150,1725)     6   (675,1725)   <- 障碍
 *    7   (1200,1725)    8   (1725,1725)  <- 障碍
 *    9   (2250,1725)
 *   10   (150,1200)    11   (675,1200)
 *   12   (1200,1200)   13   (1725,1200)
 *   14   (2250,1200)
 *   15   (150,625)     16   (675,625)    <- 障碍
 *   17   (1200,625)    18   (1725,625)   <- 障碍
 *   19   (2250,625)
 *   20   (150,150)     21   (675,150)
 *   22   (1200,150)    23   (1725,150)
 *   24   (2250,150)
 * ----------------------------------------------------------------------------
 */
static const u16 NODE_X[25] = {
    150,  675, 1200, 1725, 2250,
    150,  675, 1200, 1725, 2250,
    150,  675, 1200, 1725, 2250,
    150,  675, 1200, 1725, 2250,
    150,  675, 1200, 1725, 2250
};
static const u16 NODE_Y[25] = {
    2250, 2250, 2250, 2250, 2250,
    1725, 1725, 1725, 1725, 1725,
    1200, 1200, 1200, 1200, 1200,
     625,  625,  625,  625,  625,
     150,  150,  150,  150,  150
};

/* ----------------------------------------------------------------------------
 * 全局障碍数组：obstacle[i]=1 表示第 i 个点禁止经过。
 * 默认把 4 个交叉口设为障碍：索引 6(675,1725)、8(1725,1725)、
 *                            16(675,625)、18(1725,625)。
 * 想再禁止别的点，就把对应位置改成 1（例如把点 2 也禁止：obstacle[2]=1;）。
 * ----------------------------------------------------------------------------
 */
u8 obstacle[25] = {0};

/* 调用 path_plan 后，这里存放实际找到的关键点个数 */
u8 key_point_num = 0;

/* 由坐标找节点索引，找不到返回 -1 */
static int find_node(u16 x, u16 y){
    int i;
    for(i = 0; i < 25; i++)
        if(NODE_X[i] == x && NODE_Y[i] == y) return i;
    return -1;
}

static void mark_obstacle(u16 x, u16 y){
    int node = find_node(x, y);
    if(node >= 0) obstacle[node] = 1;
}

void path_planner_init(void){
    int i;
    for(i = 0; i < 25; i++) obstacle[i] = 0;

#if PATH_OBSTACLE_COUNT > 0
    mark_obstacle(PATH_OBSTACLE_0_X, PATH_OBSTACLE_0_Y);
#endif
#if PATH_OBSTACLE_COUNT > 1
    mark_obstacle(PATH_OBSTACLE_1_X, PATH_OBSTACLE_1_Y);
#endif
#if PATH_OBSTACLE_COUNT > 2
    mark_obstacle(PATH_OBSTACLE_2_X, PATH_OBSTACLE_2_Y);
#endif
#if PATH_OBSTACLE_COUNT > 3
    mark_obstacle(PATH_OBSTACLE_3_X, PATH_OBSTACLE_3_Y);
#endif
#if PATH_OBSTACLE_COUNT > 4
    mark_obstacle(PATH_OBSTACLE_4_X, PATH_OBSTACLE_4_Y);
#endif
#if PATH_OBSTACLE_COUNT > 5
    mark_obstacle(PATH_OBSTACLE_5_X, PATH_OBSTACLE_5_Y);
#endif
#if PATH_OBSTACLE_COUNT > 6
    mark_obstacle(PATH_OBSTACLE_6_X, PATH_OBSTACLE_6_Y);
#endif
#if PATH_OBSTACLE_COUNT > 7
    mark_obstacle(PATH_OBSTACLE_7_X, PATH_OBSTACLE_7_Y);
#endif
}

/* Dijkstra 最短路（每条边权 1），跳过障碍点 */
static void dijkstra(int src, int prev[]){
    const int INF = 0x3f3f3f3f;
    int dist[25];
    u8 used[25];
    int i, k, u, v, r, c;

    for(i = 0; i < 25; i++){ dist[i] = INF; used[i] = 0; prev[i] = -1; }
    dist[src] = 0;

    for(k = 0; k < 25; k++){
        u = -1;
        for(i = 0; i < 25; i++)
            if(!used[i] && dist[i] < INF && (u == -1 || dist[i] < dist[u])) u = i;
        if(u == -1) break;
        used[u] = 1;

        r = u / 5;   /* 行 */
        c = u % 5;   /* 列 */
        /* 上、下、左、右四个邻居（u-5 / u+5 / u-1 / u+1） */
        if(r > 0){ v = u - 5; if(!used[v] && !obstacle[v] && dist[u]+1 < dist[v]){ dist[v] = dist[u]+1; prev[v] = u; } }
        if(r < 4){ v = u + 5; if(!used[v] && !obstacle[v] && dist[u]+1 < dist[v]){ dist[v] = dist[u]+1; prev[v] = u; } }
        if(c > 0){ v = u - 1; if(!used[v] && !obstacle[v] && dist[u]+1 < dist[v]){ dist[v] = dist[u]+1; prev[v] = u; } }
        if(c < 4){ v = u + 1; if(!used[v] && !obstacle[v] && dist[u]+1 < dist[v]){ dist[v] = dist[u]+1; prev[v] = u; } }
    }
}

/* ----------------------------------------------------------------------------
 * 核心函数：求 (x0,y0) 到 (x1,y1) 的关键点路径，自动避开障碍点。
 * 输出依次经过的关键点（起点 + 每个拐弯点 + 终点）。
 * 调用后 key_point_num = 实际关键点个数；0 表示无路径或参数非法。
 * ----------------------------------------------------------------------------
 */
void path_plan(u16 x0, u16 y0, u16 x1, u16 y1, u8 n, u16 *xpath, u16 *ypath){
    int src, dst;
    int prev[25];
    int path[25], plen = 0;
    int tp[25], tlen = 0;
    int i, j, cur, t;

    key_point_num = 0;

    if(n == 0 || xpath == 0 || ypath == 0) return;

    src = find_node(x0, y0);
    dst = find_node(x1, y1);
    if(src < 0 || dst < 0) return;                 /* 坐标不对应任何网格点 */
    if(obstacle[src] || obstacle[dst]) return;     /* 起点/终点是障碍 */

    dijkstra(src, prev);
    if(prev[dst] == -1 && src != dst) return;      /* 不可达 */

    /* 重建完整路径 src -> ... -> dst */
    cur = dst;
    while(cur != -1){
        path[plen++] = cur;
        if(cur == src) break;
        cur = prev[cur];
    }
    for(i = 0, j = plen - 1; i < j; i++, j--){ t = path[i]; path[i] = path[j]; path[j] = t; }

    /* 提取关键点：起点 + 每个拐弯点 + 终点 */
    tp[tlen++] = path[0];
    for(i = 1; i < plen - 1; i++){
        int d1 = (NODE_X[path[i]] == NODE_X[path[i-1]]);   /* 1 = 竖边 */
        int d2 = (NODE_X[path[i]] == NODE_X[path[i+1]]);   /* 1 = 竖边 */
        if(d1 != d2) tp[tlen++] = path[i];                 /* 方向变了 → 拐弯点 */
    }
    if(plen > 1) tp[tlen++] = path[plen - 1];

    /* Do not write a partial route into a caller-provided buffer. */
    if(tlen > n) return;

    /* 写入输出（不超过容量 n） */
    for(i = 0; i < tlen; i++){
        xpath[i] = NODE_X[tp[i]];
        ypath[i] = NODE_Y[tp[i]];
    }
    key_point_num = (u8)tlen;
}

/* ----------------------------------------------------------------------------
 * PC 上测试用的小 demo：编译时加 -DPATH_PLANNER_DEMO 即可运行
 *   gcc -DPATH_PLANNER_DEMO path_planner.c -o path_planner_test.exe
 * ----------------------------------------------------------------------------
 */
#ifdef PATH_PLANNER_DEMO
#include <stdio.h>
int main(void){
    u16 xp[MAX_KEY_POINTS], yp[MAX_KEY_POINTS];
    u8 i;

    path_plan(150, 2250, 675, 150, MAX_KEY_POINTS, xp, yp);
    printf("key_point_num = %d\n", key_point_num);
    for(i = 0; i < key_point_num; i++)
        printf("  (%d, %d)\n", xp[i], yp[i]);
    return 0;
}
#endif
