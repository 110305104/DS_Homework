#include <iostream>
#include <vector>
#include <list>
#include <cmath>
#include <stdlib.h>
#include <algorithm>
using namespace std;


class ArrayOfSortedArrays {
public:
	
	void Insert(int num);	// 在 ArrayOfSortedArrays 中插入數字
	
	bool Search(int num);	// 在 ArrayOfSortedArrays 中搜尋數字
	
	void Print();	// 印出 ArrayOfSortedArrays 目前資料儲存的狀態(檢查用)

private:
	vector<int> arrays_[30];	//開一個內含30個vector的vector
	int flag[30] = { 0 };	// 用flag[k]紀錄地k個vector中有無存放資料
};



void ArrayOfSortedArrays::Insert(int num) {

	vector<int> tmp = { num };	// 將數字放入大小為 1 的暫存陣列中

	int k = 0;
	while (flag[k] != 0)	// 檢查flag[k]，若非0表示該vector中有資料
	{
		//將vector的資料和暫存陣列merge，儲存於暫存陣列
                   vector<int> tmp_copy = tmp;
		tmp.resize(pow(2, k + 1));
		merge(arrays_[k].begin(), arrays_[k].end(), tmp_copy.begin(), tmp_copy.end(), tmp.begin());	

		arrays_[k].clear();	//清空vector
		flag[k]--;	//更新flag[k]的狀態(無資料)
		k++;              //往下一個位置檢查
	}

	arrays_[k] = tmp;	// 若第k個vector中沒有資料，把暫存陣列的資料放過來
	flag[k]++;	//更新flag[k]的狀態(有資料)，結束此次insert

}

bool ArrayOfSortedArrays::Search(int num) {

	for (const auto& array : arrays_) {
		// 從長度為2^0的array開始，對每個array使用二分搜尋
		if (binary_search(array.begin(), array.end(), num))
			return true;	//有找到即結束搜尋並回傳true
	}
	return false;	//全部找完都沒有，回傳false
}

void ArrayOfSortedArrays::Print() {//印出儲存狀態，檢查用
         cout << "-----------------------------------------\n";
	for (int i = 0; i < 30; i++)
	{
		for (int j = 0; j < arrays_[i].size(); j++)
		{
			cout << arrays_[i][j] << " ";
		}
		cout << endl;
	}
         cout << "-----------------------------------------\n";

}


//測試
void Test()
{
	ArrayOfSortedArrays Try;

	for (int i = 0; i < 30; i++)
	{
		Try.Insert(rand() % 50 + 1);
	}


	Try.Print();

	if (Try.Search(60))
		printf("yes\n");
	else
		printf("no\n");

}

//實驗
int main() {
	srand(time(NULL));

	int start, end, repeat;
	cin >> start >> end >> repeat;

	cout << "重複 " << repeat << " 次取平均" << "\n";
	for (int k = start; k < (end + 1); k++)
	{
		double addTotalSpendTime = 0;
		double searchTotalSpendTime = 0;
		for (int n = 0; n < repeat; n++)
		{
			ArrayOfSortedArrays myAOSA;//initial Array of sorted arrays

			double START, END;
			START = clock();

			for (int i = 0; i < pow(2, k); i++)
			{
				myAOSA.Insert(rand() % 1073741824 + 1);//update Array of sorted arrays
			}

			END = clock();

			addTotalSpendTime += ((END - START) / CLOCKS_PER_SEC);


			double search_START, search_END;
			search_START = clock();
			for (int i = 0; i < pow(10, 5); i++)
			{
				myAOSA.Search(rand() % 1073741824 + 1);//search in Array of sorted arrays
			}
			search_END = clock();
			searchTotalSpendTime += ((search_END - search_START) / CLOCKS_PER_SEC);
		}
		double addAvrgSpendTime = addTotalSpendTime / repeat;
		double searchAvrgSpendTime = searchTotalSpendTime / repeat;
		cout << endl << "AOSA新增2^" << k << "個隨機數所需的時間:" << addAvrgSpendTime << " sec" << endl;
		cout << endl << "在存了2^" << k << "筆資料的AOSA中搜尋十萬筆資料所需的時間:" << searchAvrgSpendTime << " sec" << endl << endl;
	}
	return 0;
}


//聽完老師上課給的ASA實作提示後，想到 insert function 內可以改成跟老師一樣，
//用.empty檢查是否有位置能放，這樣就不用另外建和維護flag陣列了 : 

/*
void ArrayOfSortedArrays::Insert(int num) {

	vector<int> tmp = { num };	// 將數字放入大小為 1 的暫存陣列中

	int k = 0;
	while (!arrays_[k].empty())	// 當長度為2^k的vector中有資料
	{
                  //將vector的資料和暫存陣列merge，儲存於暫存陣列
		vector<int> tmp_copy = tmp;
		tmp.resize(pow(2, k + 1));
		merge(arrays_[k].begin(), arrays_[k].end(), tmp_copy.begin(), tmp_copy.end(), tmp.begin());	
                 
                  //清空vector，往下一個檢查
		arrays_[k].clear();			
                   k++;
	}

	arrays_[k] = tmp;	// 長度為2^k的vector中沒有資料，把暫存陣列的資料放過來

}
*/