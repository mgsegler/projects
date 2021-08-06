//Fizzbuzz

#include <stdio.h>

int main()
{
  for (int i = 0; i <= 100; i++)
  {
    if(i % 3 == 0)
    {
      printf("%d Fizz\n", i);
    }
    else if(i % 5 == 0)
    {
      printf("%d Buzz\n", i);
    }
    else if(i % 3 == 0 && i % 5 == 0)
    {
      printf(" d FizzBuzz!\n");
    }
    else
    {
      printf("%d\n", i);
    }
  }

  return 0;
}
