#define TargetTemp 25.00

// 定义模糊集合
typedef struct {
    float low;
    float medium;
    float high;
} FuzzyMembership;
// 模糊规则库
static const float kp_rules[3][3] = {
    // ec: SMALL, MEDIUM, LARGE
    {0.2f, 1.0f, 1.8f}, // e: SMALL
    {2.0f, 2.5f, 3.0f}, // e: MEDIUM
    {3.0f, 4.0f, 6.0f}  // e: LARGE
};
static const float ki_rules[3][3] = {
    {0.03f, 0.08f, 0.14f},
    {0.08f, 0.12f, 0.18f},
    {0.14f, 0.18f, 0.2f}};
static const float kd_rules[3][3] = {
    {0.0f, 0.5f, 0.7f},
    {0.5f, 0.8f, 1.0f},
    {0.7f, 1.0f, 1.2f}};
FuzzyMembership fuzzify_temperature(float temperature);
float defuzzify(FuzzyMembership control_membership);
void fuzzyControl(float temperature,float e,float deltae);
FuzzyMembership infer_control(FuzzyMembership error_membership, FuzzyMembership delta_error_membership);
FuzzyMembership fuzzify_delta_error(float delta_error);
FuzzyMembership fuzzify_error(float error);
float triangle(float x, float a, float b, float c);
void infer_pid(FuzzyMembership error_membership, FuzzyMembership delta_error_membership, float *delta_kp, float *delta_ki, float *delta_kd);

