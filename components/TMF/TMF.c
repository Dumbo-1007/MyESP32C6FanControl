#include <stdio.h>
#include <math.h>
#include "TMF.h"

// 三角隶属度函数x：当前输入值，用于计算该点的隶属度；
// a：三角形隶属度函数的左边界；
// b：三角形的顶点位置（最大值为 1）；
// c：三角形的右边界。
float triangle(float x, float a, float b, float c)
{
  if (x <= a || x >= c) return 0.0;
  else if (x > a && x <= b) return (x - a) / (b - a);
  else return (c - x) / (c - b);
}

// 模糊化误差
FuzzyMembership fuzzify_error(float error)
{
  FuzzyMembership membership = {0.0, 0.0, 0.0};
  float e = (error < 0) ? 0 : error; // 负数直接归零
  membership.low = triangle(e, 0.0, 0.0, 10.0);
  membership.medium = triangle(e, 5.0, 10.0, 15.0);
  membership.high = triangle(e, 10.0, 20.0, 20.0);
  return membership;
}
// 模糊化误差变化
FuzzyMembership fuzzify_delta_error(float delta_error)
{
  FuzzyMembership membership = {0.0, 0.0, 0.0};
  float de = (delta_error < 0) ? 0 : delta_error; // 负数直接归零
  membership.high = triangle(de, 2.5, 5.0, 5.0);
  membership.medium = triangle(de, 1.25, 2.5, 3.75);
  membership.low = triangle(de, 0.0, 0.0, 2.5);
  return membership;
}

// 模糊规则推理
void infer_pid(FuzzyMembership error_membership, 
  FuzzyMembership delta_error_membership, float *delta_kp, float *delta_ki, float *delta_kd)
{
  float kp = 0.0f, ki = 0.0f, kd = 0.0f; // 初始值设为0
  float total_weight = 0.0f;

  float e_memberships[3] = {error_membership.low, error_membership.medium, error_membership.high};
  float ec_memberships[3] = {delta_error_membership.low, delta_error_membership.medium, delta_error_membership.high};

  for (int e = 0; e < 3; e++)
  {
    for (int ec = 0; ec < 3; ec++)
    {
      float weight = e_memberships[e] * ec_memberships[ec];
      kp += weight * kp_rules[e][ec];
      ki += weight * ki_rules[e][ec];
      kd += weight * kd_rules[e][ec];
      total_weight += weight;

      // 添加调试日志
     // printf("e: %d, ec: %d, weight: %.2f, kp_rule: %.2f, ki_rule: %.2f, kd_rule: %.2f\n", 
            // e, ec, weight, kp_rules[e][ec], ki_rules[e][ec], kd_rules[e][ec]);
    }
  }

  // 解模糊化（加权平均）
  *delta_kp = (total_weight > 0) ? kp / total_weight : 0.0f;
  *delta_ki = (total_weight > 0) ? ki / total_weight : 0.0f;
  *delta_kd = (total_weight > 0) ? kd / total_weight : 0.0f;

  // 添加解模糊化结果日志
  //printf("total_weight: %.2f, delta_kp: %.2f, delta_ki: %.2f, delta_kd: %.2f\n", 
         //total_weight, *delta_kp, *delta_ki, *delta_kd);
}

FuzzyMembership infer_control(FuzzyMembership error_membership, FuzzyMembership delta_error_membership)
{
  FuzzyMembership control_membership = {0.0, 0.0, 0.0};

  // 规则 1: 如果误差低且误差变化低，则输出低
  control_membership.low = fmax(control_membership.low, fmax(error_membership.low, delta_error_membership.low));

  // 规则 2: 如果误差中等且误差变化中等，则输出中等
  control_membership.medium = fmax(control_membership.medium, fmax(error_membership.medium, delta_error_membership.medium));

  // 规则 3: 如果误差高且误差变化高，则输出高
  control_membership.high = fmax(control_membership.high, fmax(error_membership.high, delta_error_membership.high));

  // 规则 4: 如果误差高且误差变化低，则输出高
  control_membership.high = fmax(control_membership.high, fmax(error_membership.high, delta_error_membership.low));

  // 规则 5: 如果误差低且误差变化高，则输出低
  control_membership.low = fmax(control_membership.low, fmax(error_membership.low, delta_error_membership.high));

  return control_membership;
}
// 去模糊化函数
float defuzzify(FuzzyMembership control_membership)
{
  float numerator = 0.0;
  float denominator = 0.0;

  // 低功率部分
  for (float x = 0.0; x <= 50; x += 1.0)
  {
    float membership = fmin(control_membership.low, triangle(x, 0.0, 0.0, 50));
    numerator += x * membership;
    denominator += membership;
  }

  // 中等功率部分
  for (float x = 25; x <= 75; x += 1.0)
  {
    float membership = fmax(control_membership.medium, triangle(x, 25, 50, 75));
    numerator += x * membership;
    denominator += membership;
  }

  // 高功率部分
  for (float x = 50; x <= 100.0; x += 1.0)
  {
    float membership = fmax(control_membership.high, triangle(x, 50, 100.0, 100.0));
    numerator += x * membership;
    denominator += membership;
  }

  // 如果分母为 0，返回 0；否则返回比例缩放后的值
  float output = (denominator == 0.0) ? 0.0 : (numerator / denominator);

  // 将输出从 [0, 100] 映射到 [0, 255]
  return output * 255.0 / 100.0;
}

