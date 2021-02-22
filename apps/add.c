int add(int n) {
    return n <= 1 ? 1 : n + add(n - 1);
}

int main() {
    int a = 4;
    int b = 5;
    int c = a + b;
    int d = add(c);
    return 0;
}
