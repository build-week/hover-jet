//%ignore
#include <iostream>
#include <vector>

template <typename T>
class OutputArg : T {
  OutputArg(T& data) {
    data_ = data;
  }
  operator T() {
    return data_;
  }

  T& data_;
};

template <typename T>
T& is_output(T& arg) {
  return arg;
}

void add(const int a, const int b, std::vector<int>& c) {
  c.push_back(a + b);
}

int main() {
  const int        a = 3;
  const int        b = 1;
  std::vector<int> c;
  add(a, b, is_output(c));
  std::cout << c.size() << std::endl;
  if (!c.empty()) {
    std::cout << c[0] << std::endl;
  }
}