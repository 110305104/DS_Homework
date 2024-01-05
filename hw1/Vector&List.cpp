#include <iostream>
#include <stdlib.h>
#include <vector>
#include <list>
#include <cmath>
#include <time.h>


using namespace std;

void vectorTest() {
    srand(time(NULL));

    int start, end, repeat;
    cin >> start >> end >> repeat;

    cout << "重複 " << repeat << " 次取平均" << "\n";
    for (int k = start; k < (end+1); k++)
    {
        double addTotalSpendTime = 0;
        double sumTotalSpendTime = 0;
        for (int n = 0; n < repeat; n++)
        {
            vector<int> dynamicArray;
            double START, END;
            START = clock();
            for (int i = 0; i < pow(2, k); i++)
            {
                dynamicArray.push_back(rand());
            }
            END = clock();

            addTotalSpendTime += ((END - START) / CLOCKS_PER_SEC);


            double sum_START, sum_END;
            sum_START = clock();
            int sum = 0;
            for (int i = 0; i < pow(2, k); i++)
            {
                sum += dynamicArray[i]; //若單純要測Traverse time把這行註解掉
            }
            sum_END = clock();
            sumTotalSpendTime += ((sum_END - sum_START) / CLOCKS_PER_SEC);
        }
        double addAvrgSpendTime = addTotalSpendTime / repeat;
        double sumAvrgSpendTime = sumTotalSpendTime / repeat;
        cout << endl << "Dynamic Array新增2^" << k << "個隨機數所需的時間:" << addAvrgSpendTime << " sec" << endl;
        cout << endl << "Dynamic Array新增2^" << k << "個隨機數後計算總合所需的時間:" << sumAvrgSpendTime << " sec" << endl << endl;
    }

}

void listTest() {
    srand(time(NULL));

    int start, end, repeat;
    cin >> start >> end >> repeat;

    cout << "重複 " << repeat << " 次取平均" << "\n";
    for (int k = start; k < (end+1); k++)
    {
        double addTotalSpendTime = 0;
        double sumTotalSpendTime = 0;
        for (int n = 0; n < repeat; n++)
        {
            list<int> linkedList;
            double START, END;
            START = clock();
            for (int i = 0; i < pow(2, k); i++)
            {
                linkedList.push_back(rand());
            }
            END = clock();

            addTotalSpendTime += ((END - START) / CLOCKS_PER_SEC);


            double sum_START, sum_END;
            sum_START = clock();
            int sum = 0;
            for (int n : linkedList)
            {
                sum += n; //若單純要測Traverse time把這行註解掉
            }
            sum_END = clock();
            sumTotalSpendTime += ((sum_END - sum_START) / CLOCKS_PER_SEC);
        }
        double addAvrgSpendTime = addTotalSpendTime / repeat;
        double sumAvrgSpendTime = sumTotalSpendTime / repeat;
        cout << endl << "Linked List新增2^" << k << "個隨機數所需的時間:" << addAvrgSpendTime << " sec" << endl;
        cout << endl << "Linked List新增2^" << k << "個隨機數後計算總合所需的時間:" << sumAvrgSpendTime << " sec" << endl << endl;
    }

}


int main()
{
    string dataStruc;
    cin >> dataStruc;
    if (dataStruc == "vector")
    {
        vectorTest();
    }
    else
    {
        listTest();
    }
    return 0;
}