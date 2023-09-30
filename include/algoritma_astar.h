#ifndef _algoritma_astar_h_
#define _algoritma_astar_h_

#include <Arduino.h>
#include <bits/stdc++.h>
#include <USlib.h>
#include <SD.h>
#include "mbedtls/md.h"



using namespace std; 

class algoritma_astar
{
private:
    /* data */
    uint16_t X_kotak = 0, Y_kotak = 0;

    USlib _US;

    vector<vector<uint32_t>> extractdata(vector<vector<float>> data, vector<vector<uint32_t>> titiks);
    vector<vector<vector<uint32_t>>> titiksudut(vector<vector<float>> data);
    void print(vector<vector<int32_t>> XY);
    vector<vector<int32_t>> linier(vector<vector<uint32_t>> titik);
    vector<vector<int32_t>> expansion_linier(vector<vector<int32_t>> XY);
    float keliling_oval();
    vector<vector<int32_t>> cvt_biner(vector<vector<vector<int32_t>>> XY);

    // uint16_t Tinggi_Muka = keliling_oval();
    // const int ROW = (_Y_max/Tinggi_Muka)+1;
    // const int COL = (_X_max/Tinggi_Muka)+1;
    int ROW = 0;
    int COL = 0;

    String TRAIN_filename_path = "/data-training/path.csv";
    String TRAIN_filename_barang = "/data-training/barang.csv";

    // Creating a shortcut for int, int pair type 
    typedef pair<int, int> Pair; 

    // Creating a shortcut for pair<int, pair<int, int>> type 
    typedef pair<double, pair<int, int>> pPair; 

    // A structure to hold the neccesary parameters 
    struct cell 
    { 
        // Row and Column index of its parent 
        // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1 
        int parent_i, parent_j; 
        // f = g + h 
        double f, g, h; 
    }; 

    int8_t feedback = 0;

    bool manual_dataset = 1; //true manual

    bool isValid(int row, int col);
    bool isUnBlocked(vector<vector<int>> grid, int row, int col);
    bool isDestination(int row, int col, Pair dest);
    double calculateHValue(int row, int col, Pair dest);
    void tracePath(vector<vector<cell>> cellDetails, Pair dest, int8_t arah);
    void aStarSearch(vector<vector<int>>, Pair src, Pair dest);
    vector<vector<int32_t>> binertoint(String filename);

public:
	/*
        Cell-->Popped Cell (i, j) 
		1. N --> North	 (i-1, j) 
		2. S --> South	 (i+1, j) 
		3. E --> East	 (i, j+1) 
		4. W --> West		 (i, j-1) 
		5. N.E--> North-East (i-1, j+1) 
		6. N.W--> North-West (i-1, j-1) 
		7. S.E--> South-East (i+1, j+1) 
		8. S.W--> South-West (i+1, j-1)
    */
    vector<vector<int16_t>> Paths;
    uint16_t X_max, Y_max, Lebar_Muka, lingkar_badan; // in CM
    uint16_t Tinggi_Muka = 0;

    // filename         : untuk mengisi nama file data sensor
    // X_max           : lebar dari kamar
    // Y_max           : Panjang dari kamar
    // Lebar_Muka      : lebar dari lingkar badan manusia
    // lingkar_badan   : keliling dari lingkar badan manusia 
    void init(String filename, uint16_t _X_max, uint16_t _Y_max, uint16_t _Lebar_Muka = 33, uint16_t _lingkar_badan = 90);


    // src_X            : Source X
    // src_Y            : Source Y
    // dst_X            : Destination X
    // dst_Y            : Destination Y
    int8_t Search(String filename, uint16_t src_X, uint16_t src_Y, uint16_t dst_X, uint16_t dst_Y);

    // filename         : untuk mengisi nama file data barang
    vector<vector<int32_t>> Tujuan_koordinat(String filename, String filename_map);

};




#endif