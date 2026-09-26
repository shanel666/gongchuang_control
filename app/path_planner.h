/* ============================================================================
 * path_planner.h —— 21 途经点路径规划（含 4 个障碍交叉口）
 *
 * 【怎么用】
 *   1) 把 path_planner.c 和 path_planner.h 一起加进你的工程；
 *   2) 如果你的工程里已经 typedef 过 u8 / u16，请删掉下面「类型定义」那几行；
 *   3) 设置障碍点：全局数组 obstacle[i] = 1 表示第 i 个点禁止经过；
 *      默认已把 4 个交叉口设为障碍（见 path_planner.c 里的坐标表）；
 *   4) 调用：
 *         u16 xp[25], yp[25];
 *         path_plan(150, 2250, 675, 150, 25, xp, yp);
 *      调用后：
 *         key_point_num = 实际关键点个数（0 表示无路径 / 参数非法）；
 *         xp[0..key_point_num-1] / yp[...] = 依次经过的关键点坐标（含起点终点）。
 *
 * 【参数说明】
 *   (x0,y0)  起点坐标（你的原始坐标系，左下角为原点）
 *   (x1,y1)  终点坐标
 *   n        xpath / ypath 数组的容量（最多能存几个点），用来防止越界
 *   xpath    输出：依次经过的关键点的 x 坐标
 *   ypath    输出：依次经过的关键点的 y 坐标
 *
 * 【说明】
 *   本函数是 void，且 n 是按值传入的，所以「实际关键点个数」用全局变量
 *   key_point_num 返回。
 *   如果你更希望 n 本身就是输出（函数里 *n = 个数），把签名改成：
 *       void path_plan(u16 x0,u16 y0,u16 x1,u16 y1, u8 *n, u16 *xpath, u16 *ypath);
 *   并在函数末尾把 tlen 写进 *n 即可。
 * ============================================================================ */

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include "stm32f10x.h"                  // Device header



/* 关键点数组最大容量（示例 / 演示用；实际最大不会超过 25） */
#define MAX_KEY_POINTS 25

/* 全局障碍数组：obstacle[i]=1 表示第 i 个点禁止经过（共 25 个网格交点） */
extern u8 obstacle[25];
void path_planner_init(void);

/* 调用 path_plan 后，这里存放实际找到的关键点个数（0 = 无路径或参数非法） */
extern u8 key_point_num;

/* 核心函数 */
void path_plan(u16 x0, u16 y0, u16 x1, u16 y1, u8 n, u16 *xpath, u16 *ypath);

#endif /* PATH_PLANNER_H */
