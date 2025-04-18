#include <string>
#include <iostream>

namespace robolab
{
    class Robot
    {
    public:
        Robot(std::string name)
        {
            _name = name;
        }

        void tellMeSomething()
        {
            std::cout << "Ich heiße " << _name << std::endl;
        }

    private:
        std::string _name;
    };
} // end namespace

int main(int argc, char *argv[])
{
    robolab::Robot r("R2D2");
    r.tellMeSomething();
}
