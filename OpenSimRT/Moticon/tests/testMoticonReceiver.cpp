#include "MoticonReceiver.h"

#include <iostream>
using namespace std;

void run() {
        MoticonReceiver moticon;

        while (true) {
            auto output = moticon.receiveData();
            cout << output.left.totalForce << " " << output.right.totalForce << endl;
        }
}

int main(int argc, char* argv[]) {
    try {
        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
}