#include <iostream>

using namespace std;

class P1
{
    public:
    int a;
    int b;
    P1(int x, int y):a(x),b(y){}
};

class P2
{
    public:
    int c;
    float d;
    P2(int x, int y):c(x),d(y){}
};

int main()
{
    P1 p1(1,2);
    P2 p2(3,4.33);
    void *ptrs[] = {&p1.a,&p1.b,&p2.c, &p2.d};
    cout << *((int *)ptrs[0]);
}