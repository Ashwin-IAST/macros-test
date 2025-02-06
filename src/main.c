#include "config.h"
#include "utils.h"

int main() {
    LOG("Application is starting...");
    
    printf("Version: %f\n", VERSION);
    printf("Buffer Size: %d\n", BUFFER_SIZE);

    int result1 = add(5, 3);
    int result2 = subtract(10, 4);

    printf("Addition Result: %d\n", result1);
    printf("Subtraction Result: %d\n", result2);

    print_version();

    return 0;
}

void print_version() {
    printf("Software Version: %f\n", VERSION);
}
