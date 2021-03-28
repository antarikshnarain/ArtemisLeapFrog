#include <iostream>

using namespace std;

class Prop
{
    int a;
    int b;
    public:
    Prop(int x,int y):a(x),b(y){}

    void operator<<(const Prop &p)
    {
        this->a = p.a;
        this->b = p.b;
    }
    void print()
    {
        cout << a << '\t' <<b << endl;
    }
};

int main()
{
    Prop p1(2,3),p2(4,5);
    p1.print();
    p2.print();
    p1 << p2;
    p1.print();
    p2.print();
}