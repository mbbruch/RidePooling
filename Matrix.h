#pragma once
#include<iostream>
#include "globals.h"


struct Matrix
{
	int n;//Matrix length and width
	int** a;

	Matrix() :n(0), a(NULL) {}

	~Matrix() { clear(); }

	void save()
	{
		printf("%d\n", n);
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
				printf("%d ", a[i][j]);
			printf("\n");
		}
	}

	void load()
	{
		scanf("%d", &n);
		a = new int* [n];
		for (int i = 0; i < n; i++)a[i] = new int[n];
		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
				scanf("%d", &a[i][j]);
	}

	void cover(int x)
	{
		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
				a[i][j] = x;
	}

	void init(int N)
	{
		clear();
		n = N;
		a = new int* [n];
		for (int i = 0; i < n; i++)a[i] = new int[n];
		for (int i = 0; i < n; i++)
			for (int j = 0; j < n; j++)
				a[i][j] = INF;
		for (int i = 0; i < n; i++)a[i][i] = 0;
	}

	void clear()
	{
		for (int i = 0; i < n; i++)delete[] a[i];
		delete[] a;
	}

	void floyd(Matrix& b)
	{
		int i, j, k;
		for (k = 0; k < n; k++)
			for (i = 0; i < n; i++)
				for (j = 0; j < n; j++)
					if (a[i][j] > a[i][k] + a[k][j]) {
						a[i][j] = a[i][k] + a[k][j];
						b.a[i][j] = b.a[i][k] + b.a[k][j];
					}
	}

	void floyd(Matrix& order, Matrix& b)//Floyd the matrix a and record the scheme in order
	{
		int i, j, k;
		for (k = 0; k < n; k++)
			for (i = 0; i < n; i++)
				for (j = 0; j < n; j++)
					if (a[i][j] > a[i][k] + a[k][j])
					{
						a[i][j] = a[i][k] + a[k][j];
						b.a[i][j] = b.a[i][k] + b.a[k][j];
						order.a[i][j] = k;
					}
	}

	void write()
	{
		printf("n=%d\n", n);
		for (int i = 0; i < n; i++, cout << endl)
			for (int j = 0; j < n; j++)printf("%d ", a[i][j]);
	}

	Matrix& operator =(const Matrix& m)
	{
		if (this != (&m))
		{
			init(m.n);
			for (int i = 0; i < n; i++)
				for (int j = 0; j < n; j++)
					a[i][j] = m.a[i][j];
		}
		return *this;
	}
};
