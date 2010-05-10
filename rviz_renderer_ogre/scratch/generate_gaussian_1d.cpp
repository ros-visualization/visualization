#include <cmath>
#include <cstdio>
#include <cstdlib>

#define E 2.71828183
#define PI 3.14159265

double g(double x, double stddev)
{
  return (1.0 / (sqrt(2 * PI) * stddev)) * pow(E, -((x*x)/(2*stddev*stddev)));
}

int main(int argc, char** argv)
{
  int bound = atoi(argv[1]);

  double sum = 0;

  for (int i = -bound; i <= bound; ++i)
    {
      double val = g(i, atof(argv[2]));
      printf("%f,\n", val);

      sum += val;
    }

  printf("sum: %f\n", sum);
}
