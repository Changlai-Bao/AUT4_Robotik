#include <iostream>
#include <string>

namespace robolab
{
    template <class T>
    class Robot
    {
    public:
        Robot(std::string name, T secret)
        {
            _name = name;
            _secret = secret;
        }

        void tellMeSomething()
        {
            std::cout << "Ich heiße " << _name;
            std::cout << ". Mein Geheimnis ist: " << _secret << std::endl;
        }

    private:
        std::string _name;
        T _secret;
    };
} // end namespace

int main(int argc, char *argv[])
{
    robolab::Robot<std::string> r("R2D2", "Nicht weitersagen!");
    robolab::Robot<double> r2("R2D2", 4711);
    r.tellMeSomething();
    r2.tellMeSomething();
}