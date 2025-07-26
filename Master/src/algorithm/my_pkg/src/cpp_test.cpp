#include <iostream>
#include <functional>
using namespace std;   
void foo(int& x){
    x++;
} 
void func(int& x){
    x+=2;
}
int main(){
    int x = 10;
    auto fun1=std::bind(foo,x);
    func(x);
    fun1();
    cout << "x:"<< x << std::endl;
}