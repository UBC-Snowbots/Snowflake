#include <iostream>
#include <sys/epoll.h>

class SerialDetector {
    private:
        void setup();
    public:
        /*
        SerialDetector()
        string read(prefix, num bytes)
        write()
        */
};

class SerialSession {
    private:
        SerialSession();
    public:
        std::ostream& writeStream;
        std::istream& readStream;
};

int main(){}
