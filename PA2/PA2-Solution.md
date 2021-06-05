### 熟悉Eigen矩阵运算

设线性方程 Ax = b,在 A 为方阵的前提下,请回答以下问题:

1. 在什么条件下,x 有解且唯一?

   > A 满秩的情况下，x 有解且唯一

2. 高斯消元法的原理是什么?

   > 高斯消元法是线性代数中的一种算法，可用于求解线性方程组的解，以及求出矩阵的秩，和求出可逆矩阵的逆矩阵。
   > 高斯消元法的原理是：通过用初等行变换将增广矩阵化为行阶梯阵，然后通过回带求解线性方程组的解。

3. QR 分解的原理是什么?

   > QR分解法是三种将矩阵分解的方式之一。这种方式，把矩阵分解成一个正交矩阵与一个上三角矩阵的积。QR分解经常用来解线性最小二乘法问题。
   >
   > 

4. Cholesky 分解的原理是什么?

5. 编程实现 A 为 100 × 100 随机矩阵时,用 QR 和 Cholesky 分解求 x 的程序

   > [PA2-5](./code/main.cpp#PA2_5)

### 几何运算练习

> [PA3](./code/main.cpp#PA3)

### 旋转的表达

1. 设有旋转矩阵$R$,证明 $RR^T=1$ 且 $detR=1$ 
   > $$
   > RR^T=RR^{-1}=I
   > $$
   > $$
   > det(R)=det(r_{1}, r_{2}, r_{3})=r_{1}\cdot(r_{2}\times r_{3}) = 1
   > $$
2. 四元数 $q=(\epsilon, \eta)$, 虚部 $\epsilon$ 为三维, 实部 $\eta$ 为一维
3. 四元数乘法
   
   四元数乘法写成向量形式为：
   $$
   \begin{aligned}
      q\otimes p =& \begin{bmatrix}
          p_wq_w-p_xq_x-p_yq_y-p_zq_z \\
          p_wq_x+p_xq_w+p_yq_z-p_zq_y \\
          p_wq_y-p_xq_z+p_yq_w+p_zq_x \\
          p_wq_z+p_xq_y-p_yq_x+p_zq_w
      \end{bmatrix} \\
      =& \begin{bmatrix}
          p_wq_w - \pmb{p}_v^T\pmb{q}_v \\
          p_w\pmb{q}_v+q_w\pmb{p}_v+\pmb{p}_v\times \pmb{q}_v

      \end{bmatrix}
   \end{aligned}
   $$
   四元数乘法又可以表示为矩阵乘法: 
   $$
   \pmb{q}_1\otimes \pmb{q}_2=[\pmb{q}_1]_L\pmb{q}_2 \\
   \pmb{q}_1\otimes \pmb{q}_2=[\pmb{q}_2]_R\pmb{q}_1
   $$
   根据上式：
   $$
   \begin{aligned}
      [\pmb{q}_1]_L=&\begin{bmatrix}
          q_w& -q_z& qy& q_x\\
          q_z& q_w& -q_x& q_y\\
          -q_y& q_x& q_w& q_z\\
          -q_x& -q_y& -q_z& q_w
      \end{bmatrix}\\
      =&\begin{bmatrix}
          \eta1 + \epsilon^{\wedge}& \epsilon \\
          -\epsilon& \eta
      \end{bmatrix}
   \end{aligned}
   $$
   同理可以写出$\pmb[q_2]_R$
### 罗德里格斯公式的证明



### 四元数运算性质的验证



### 熟悉 C++ 11

```cpp
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class A {
public:
    A(const int& i ) : index(i) {}
    int index = 0;
};


int main() {
    A a1(3), a2(5), a3(9);
    // c++11 列表初始化
    vector<A> avec{a1, a2, a3};
    // c++11 lambda表达式作为标准容器的比较函数
    std::sort(avec.begin(), avec.end(), [](const A&a1, const A&a2) {return a1.index<a2.index;});
    // 范围for循环， auto 自动类型推导
    for ( auto& a: avec ) cout<<a.index<<" ";
    cout<<endl;
    return 0;
}
```