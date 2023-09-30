
#include <algoritma_astar.h>

void algoritma_astar::init(String filename, uint16_t _X_max, uint16_t _Y_max, uint16_t _Lebar_Muka, uint16_t _lingkar_badan){

	X_max = _X_max, Y_max = _Y_max, Lebar_Muka = _Lebar_Muka, lingkar_badan = _lingkar_badan;

    Tinggi_Muka = keliling_oval();
    ROW = (Y_max/Tinggi_Muka)+1;
    COL = (X_max/Tinggi_Muka)+1;

	if(!manual_dataset){
		vector<vector<float>> data;
		File myFile = SD.open(filename);
		if (myFile) {
			Serial.println(filename);

			// read from the file until there's nothing else in it:
			// String txt = "";
			vector<float> m;
			String txt = "";
			while (myFile.available()) {
				// Serial.write(myFile.read());
				// txt += (char) myFile.read();
				char k = (char) myFile.read();
				if(k == ','){
					m.push_back(txt.toFloat());
					txt = "";
				} else if (k == '\n'){
					m.push_back(txt.toFloat());
					txt = "";
					data.push_back(m);
					m.clear();
				} else {
					txt += k;
				}
			}
			myFile.close();
		} else {
			// if the file didn't open, print an error:
			Serial.print("error opening ");
			Serial.println(filename);
		}

		// for (auto k : data){
		// 	for (auto l : k){
		// 		Serial.print(l);
		// 		Serial.print(" ");
		// 	}
		// 	Serial.println();
		// }

		vector<vector<vector<uint32_t>>> titiks = titiksudut(data);
		// for (auto m : titiks){
		// 	for (auto k : m){
		// 		for (auto l : k){
		// 			Serial.print(l);
		// 			Serial.print(" ");
		// 		}
		// 		Serial.println();
		// 	}
		// 	Serial.println();
		// }
		
		vector<vector<vector<int32_t>>> mxy;
		for (auto titik : titiks){
			titik = extractdata(data, titik);
			
			vector<vector<int32_t>> XY = linier(titik);
			XY = expansion_linier(XY);   	 
			mxy.push_back(XY);
		}
		vector<vector<int32_t>> XY = cvt_biner(mxy);

		// for (auto k : XY){
		// 	for (auto l : k){
		// 		Serial.print(l);
		// 		Serial.print(" ");
		// 	}
		// 	Serial.println();
		// }

		myFile = SD.open("/cache/.map_kamar", FILE_WRITE);

		// if the file opened okay, write to it:
		if (myFile) {
			Serial.print("Writing to /cache/.map_kamar...");
			for (auto i : XY){
				for (auto k : i){
					myFile.print(k);
					myFile.print(",");
				}			
				myFile.println();
			}
			// close the file:
			myFile.close();
			Serial.println("write done.");
		} else {
			// if the file didn't open, print an error:
			Serial.println("error: File map_cache not found");
			return;
		}

		String txt = "";
		myFile = SD.open("/cache/.map_kamar");
		if (myFile) {
			Serial.println("/cache/.map_kamar");

			// read from the file until there's nothing else in it:
			// String txt = "";
			String txts = "";
			while (myFile.available()) {
				// Serial.write(myFile.read());
				// txt += (char) myFile.read();
				char k = (char) myFile.read();
				if(k == ','){
					txt += String(txts); 
					txt += String(',');
					txts = "";
				} else if (k == '\n'){
					txt += String(txts); 
					txt += String('\n');
					txts = "";
				} else {
					txts += k;
				}
			}
			myFile.close();
		} else {
			// if the file didn't open, print an error:
			Serial.print("error opening ");
			Serial.println("/cache/.map_kamar");
			return;
		}
	}
	Serial.println("Astar: init done");
}

int8_t algoritma_astar::Search(String filename, uint16_t src_X, uint16_t src_Y, uint16_t dst_X, uint16_t dst_Y){

	feedback = 0;
	Paths.clear();
	vector<vector<int32_t>> XY;
	String txt = "";
	File myFile = SD.open(filename);
	if (myFile) {
		Serial.println(filename);

		// read from the file until there's nothing else in it:
		// String txt = "";
		vector<int32_t> m;
		String txts = "";
		while (myFile.available()) {
			// Serial.write(myFile.read());
			// txt += (char) myFile.read();
			char k = (char) myFile.read();
			if(k == ','){
				m.push_back(txts.toInt());
				txt += String(txts); 
				txt += String(',');
				txts = "";
			} else if (k == '\n'){
				m.push_back(txts.toFloat());
				txt += String(txts); 
				txt += String('\n');
				txts = "";
				XY.push_back(m);
				m.clear();
			} else {
				txts += k;
			}
		}
		myFile.close();
	} else {
		// if the file didn't open, print an error:
		Serial.print("error opening ");
		Serial.println(filename);
		return 3;
	}


	// Source is the left-most bottom-most corner 
	Pair src = make_pair(src_Y, src_X); 

	// Destination is the left-most top-most corner 
	Pair dest = make_pair(dst_Y, dst_X); // ROW - COL

	// cout << XY[src.first][src.second] << " " << XY[dest.first][dest.second] << endl;
	
	aStarSearch(XY, src, dest); 

	vector<vector<int16_t>> mmy;
	for (int i = 0; i < Paths.size(); i++){
		/* code */
		vector<int16_t> mk;
		if(Paths.size()-1 == i){
			mk.push_back(Paths[i][0]);
			mk.push_back(Paths[i][1]);
			mk.push_back(0);
		} else if(Paths[i][0] > Paths[i+1][0] && Paths[i][1] == Paths[i+1][1]){ // Kiri
			mk.push_back(Paths[i][0]);
			mk.push_back(Paths[i][1]);
			mk.push_back(4);
		} else if(Paths[i][0] < Paths[i+1][0] && Paths[i][1] == Paths[i+1][1]){ // Kanan
			mk.push_back(Paths[i][0]);
			mk.push_back(Paths[i][1]);
			mk.push_back(3);		
		} else if(Paths[i][0] == Paths[i+1][0] && Paths[i][1] > Paths[i+1][1]){ // Atas
			mk.push_back(Paths[i][0]);
			mk.push_back(Paths[i][1]);
			mk.push_back(1);		
		} else if(Paths[i][0] == Paths[i+1][0] && Paths[i][1] < Paths[i+1][1]){ // Bawah
			mk.push_back(Paths[i][0]);
			mk.push_back(Paths[i][1]);
			mk.push_back(2);		
		}
		mmy.push_back(mk);
	} 

	Paths.clear();
	Paths = mmy;
	return feedback;
}

vector<vector<uint32_t>> algoritma_astar::extractdata(vector<vector<float>> data, vector<vector<uint32_t>> titiks){
    vector<vector<uint32_t>> titik;

    for (auto i : titiks){
        vector<uint32_t> m;
        vector<float> l = data[i[1]];
        // bool skiped = 0;

        if(l[2] < l[3]){
            m.push_back( (X_max - l[2]) / Tinggi_Muka);
        } else {
            m.push_back(l[2]  / Tinggi_Muka);
        }        
        if(l[0] > l[1]){
            m.push_back( (Y_max - l[1])  / Tinggi_Muka);
        } else {
            m.push_back(l[1] / Tinggi_Muka);
        }

        titik.push_back(m);
    }
    
    return titik;
}

vector<vector<vector<uint32_t>>> algoritma_astar::titiksudut(vector<vector<float>> data){

    vector<vector<uint32_t>> 			titik;
	vector<vector<vector<uint32_t>>> 	myz;
    uint32_t temp = 0;
    for (int i = 0; i < data.size(); i++){
        float heading = data[i][4];
        vector<uint32_t> m;
        if(heading == 0 && data[i][0] == 0 && data[i][1] == 0 && data[i][2] == 0 && data[i][3] == 0){
            m.push_back(0);
            m.push_back(i-1);
            temp = 0;
            titik.push_back(m);
			myz.push_back(titik);
			titik.clear();
        } else if(heading > (258.0-_US.US_DERAJAT) && heading < (258.0+_US.US_DERAJAT) && temp != 1){ // 270 derajat
            m.push_back(1);
            m.push_back(i);
            temp = 1;
            titik.push_back(m);
        } else if(heading > (103.0-_US.US_DERAJAT) && heading < (103.0+_US.US_DERAJAT) && temp != 2){
            m.push_back(2);
            m.push_back(i);
            temp = 2;
            titik.push_back(m);
        } else if(heading > (188.0-_US.US_DERAJAT) && heading < (188.0+_US.US_DERAJAT) && temp != 3){
            m.push_back(3);
            m.push_back(i);
            temp = 3;
            titik.push_back(m);
        } else if(((heading > (351.0-_US.US_DERAJAT) && heading < 360.0) || (heading > 0.0 && heading < ((351.0+_US.US_DERAJAT)-360.0))) && temp != 4){
            m.push_back(4);
            m.push_back(i);
            temp = 4;
            titik.push_back(m);
        }
    }

	// vector<vector<vector<uint32_t>>> 	myz2;
	// for (auto i : myz){
	// 	for (int m = 0; m < (int)(i.size()-1); m++){
	// 		if(i[m][2] != i[m+1][2]){
	// 			myz2.push_back(i);
	// 		}
	// 	}
    // }	

	// myz.clear();
    return myz;
    
}

void algoritma_astar::print(vector<vector<int32_t>> XY){
    vector<vector<uint32_t>> mxy;
    for (int i = 0; i < (X_max/Tinggi_Muka); i++){
        vector<uint32_t> m;
        for (int k = 0; k < (Y_max/Tinggi_Muka); k++){
            m.push_back(0);
        }
        mxy.push_back(m);
    }


    for(auto l : XY){
        mxy[l[0]][l[1]] = 1;
    }

    for(auto l : mxy){
        for(auto k : l){
            cout << k << "";
        }
        cout << endl;
    }   
}

vector<vector<int32_t>> algoritma_astar::cvt_biner(vector<vector<vector<int32_t>>> XY){
    vector<vector<int32_t>> mxy;
    
	for (int i = 0; i < (X_max/Tinggi_Muka); i++){
        vector<int32_t> m;
        for (int k = 0; k < (Y_max/Tinggi_Muka); k++){
            m.push_back(0);
        }
        mxy.push_back(m);
    }

	for(auto k : XY){
		for(auto l : k){
			mxy[l[0]][l[1]] = 1;
		}
	}

    return mxy;
}

vector<vector<int32_t>> algoritma_astar::linier(vector<vector<uint32_t>> titik){

    vector<vector<int32_t>> rumus;
    for (int i = 0; i < (titik.size()-1); i++){
        /* code */
        vector<int32_t> XYC;
        int32_t     Y = (titik[i+1][0]-titik[i][0]);
        int32_t     X = (titik[i+1][1]-titik[i][1]);
        int32_t     C = X*(-titik[i][0]);
                    // cout << C << " ";
                    C -= Y*(-titik[i][1]);
                    // cout << C << endl;
        XYC.push_back(X);
        XYC.push_back(Y);
        XYC.push_back(C);
        rumus.push_back(XYC);
    }

    // for(auto l : rumus){
    //     for(auto k : l){
    //         cout << k << " ";
    //     }
    //     cout << endl;
    // }  

    vector<vector<int32_t>> XY2;
    for (int i = 0; i < rumus.size(); i++){
        uint32_t X;
        uint32_t Y;
        if(rumus[i][0] < 0){
            X = -(rumus[i][0]);
        } else {
            X = (rumus[i][0]);
        }
        if(rumus[i][1] < 0){
            Y = -(rumus[i][1]);
        } else {
            Y = (rumus[i][1]);
        }

        // cout << rumus[i][0] << " " << rumus[i][1] << " " << rumus[i][2] << endl;    
        // cout << X << " " << Y << endl;    

        if(X < Y){
            // cout << titik[i][0] << " " << titik[i+1][0] << endl;
            // cout << "1" << endl;
            if( titik[i][0] > titik[i+1][0]){             
                for (int k = titik[i][0]; k >= (int) titik[i+1][0]; k--){
                    vector<int32_t> XY;   
                    int32_t x = k;
                    int32_t y = ((rumus[i][0] * x) + rumus[i][2]) / rumus[i][1];

                    XY.push_back(x); 
                    XY.push_back(y);
                    XY2.push_back(XY);
                }
                
            } else {
                for (int k = titik[i][0]; k <= (int) titik[i+1][0]; k++){
                    /* code */
                    vector<int32_t> XY;
                    int32_t x = k;
                    int32_t y = ((rumus[i][0] * x) + rumus[i][2]) / rumus[i][1];
                    XY.push_back(x);
                    XY.push_back(y);
                    XY2.push_back(XY);
                }

            }
            
        } else if(X > Y){
            // cout << "2" << " = ";
			// cout << X << " " << Y << endl;
            if( titik[i][1] > titik[i+1][1]){
                // vector<int32_t> XY;
				// cout << titik[i][1] << " " << titik[i+1][1] << endl;				
                for (int k = titik[i][1]; k >= (int) titik[i+1][1]; k--){

                    vector<int32_t> XY;
                    int32_t y = k;
                    int32_t x = ((-rumus[i][1] * y) + rumus[i][2]) / -rumus[i][0];                   
                    XY.push_back(x);
                    XY.push_back(y);
                    XY2.push_back(XY);
                }
                
            } else {
                for (int k = titik[i][1]; k <= (int) titik[i+1][1]; k++){
                    /* code */
                    vector<int32_t> XY;
                    int32_t y = k;
                    int32_t x = ((-rumus[i][1] * y) + rumus[i][2]) / -rumus[i][0];
                    XY.push_back(x);
                    XY.push_back(y);
                    XY2.push_back(XY);
                }

            }

        }

    }
    
	

	return XY2;
}

float algoritma_astar::keliling_oval(){
    float kel = lingkar_badan;
    float BD = Lebar_Muka;

    float AC = (kel - (0.5 * 3.14 * BD)) / (0.5 * 3.14);
    X_kotak = X_max/AC;
    Y_kotak = Y_max/AC;
    return AC;
}

vector<vector<int32_t>> algoritma_astar::expansion_linier(vector<vector<int32_t>> XY){
    
	// for (auto i : XY){
	// 	for (auto m : i){
	// 		Serial.print(m);
	// 		Serial.print(" - ");
	// 	}	
	// 	Serial.println();	
	// }
	
    vector<vector<int32_t>> XYZ;
    for(int l = 0; l < (XY.size()-1); l++){
        // cout << l << " ";

        if(XY[l][0] != XY[l+1][0] && XY[l][1] != XY[l+1][1]){
            XYZ.push_back(XY[l]);
            // cout << XY[l][0] << " " << XY[l][1] << endl;
            vector<int32_t> klkl;
            klkl.push_back(XY[l+1][0]);
            klkl.push_back(XY[l][1]);
            XYZ.push_back(klkl);
            // cout << klkl[0] << " " << klkl[1] << " = 1" << endl;
            klkl.clear();
            klkl.push_back(XY[l][0]);
            klkl.push_back(XY[l+1][1]);
            XYZ.push_back(klkl);
            // cout << klkl[0] << " " << klkl[1] << " = 2" << endl;
            klkl.clear();
            klkl.push_back(XY[l+1][0]);
            klkl.push_back(XY[l+1][1]);
            XYZ.push_back(klkl);
            // cout << klkl[0] << " " << klkl[1] << " = 3" << endl;
            klkl.clear();            
        } else {
            XYZ.push_back(XY[l]);
        } 

		// Serial.print(XY[l][0]);
		// Serial.print(" ++ ");
		// Serial.print(XY[l][1]);
		// Serial.print(" - ");
		// Serial.println(l);		
    }   

    // uint8_t ekspansi = 1;
    // XY.clear();
    // for (auto m : XYZ){
    //     for (int i = m[0]; i < (m[0] + ekspansi); i++){
    //         vector<int32_t> kl;
    //         kl.push_back(i);
    //         kl.push_back(m[1]);
    //         XY.push_back(kl);        
    //     }
    //     for (int i = m[0]; i >= (m[0] - ekspansi); i--){
    //         vector<int32_t> kl;
    //         kl.push_back(i);
    //         kl.push_back(m[1]);
    //         XY.push_back(kl);                    
    //     }        
    // }
    

    return XYZ;
}


// A Utility Function to check whether given cell (row, col) 
// is a valid cell or not. 
bool algoritma_astar::isValid(int row, int col) 
{ 
	// Returns true if row number and column number 
	// is in range 

	return (row >= 0) && (row < ROW) && 
		(col >= 0) && (col < COL); 
} 

// A Utility Function to check whether the given cell is 
// blocked or not 
bool algoritma_astar::isUnBlocked(vector<vector<int>> grid, int row, int col) 
{ 
	// Returns true if the cell is not blocked else false 
	if (grid[row][col] == 1) 
		return (true); 
	else
		return (false); 
} 

// A Utility Function to check whether destination cell has 
// been reached or not 
bool algoritma_astar::isDestination(int row, int col, Pair dest) 
{ 
	if (row == dest.first && col == dest.second) 
		return (true); 
	else
		return (false); 
} 

// A Utility Function to calculate the 'h' heuristics. 
double algoritma_astar::calculateHValue(int row, int col, Pair dest) 
{ 
	// Return using the distance formula 
	return ((double)sqrt ((row-dest.first)*(row-dest.first) 
						+ (col-dest.second)*(col-dest.second))); 
} 

// A Utility Function to trace the path from the source 
// to destination 
void algoritma_astar::tracePath(vector<vector<cell>> cellDetails, Pair dest, int8_t arah) 
{ 
	printf ("\nThe Path is "); 
	int row = dest.first; 
	int col = dest.second; 

	stack<Pair> Path; 

	while (!(cellDetails[row][col].parent_i == row 
			&& cellDetails[row][col].parent_j == col )) 
	{ 
		Path.push (make_pair (row, col)); 
		int temp_row = cellDetails[row][col].parent_i; 
		int temp_col = cellDetails[row][col].parent_j; 
		row = temp_row; 
		col = temp_col; 
	} 

	Path.push (make_pair (row, col)); 
	while (!Path.empty()) 
	{ 
		pair<int,int> p = Path.top(); 
		Path.pop(); 
		// printf("-> (%d,%d) ",p.first,p.second); 
		vector<int16_t> kl;
		kl.push_back(p.second);
		kl.push_back(p.first);
		Paths.push_back(kl);
	} 

	return; 
} 

// A Function to find the shortest path between 
// a given source cell to a destination cell according 
// to A* Search Algorithm 
void algoritma_astar::aStarSearch(vector<vector<int>> grid, Pair src, Pair dest) 
{ 
	// If the source is out of range 
	if (isValid (src.first, src.second) == false) 
	{ 
		printf ("Source is invalid\n"); 
		feedback = 1;
		return; 
	} 

	// If the destination is out of range 
	if (isValid (dest.first, dest.second) == false) 
	{ 
		printf ("Destination is invalid\n"); 
		feedback = 2;
		return; 
	} 

	// Either the source or the destination is blocked 
	if (isUnBlocked(grid, dest.first, dest.second) == false){ 
		printf ("destination is blocked\n"); 
		feedback = 3;
		return; 
	} 

	if (isUnBlocked(grid, src.first, src.second) == false){ 
		printf ("Source is blocked\n"); 
		feedback = 4;
		return; 
	} 

	// If the destination cell is the same as source cell 
	if (isDestination(src.first, src.second, dest) == true) 
	{ 
		printf ("We are already at the destination\n"); 
		feedback = 5;
		return; 
	} 

	// Create a closed list and initialise it to false which means 
	// that no cell has been included yet 
	// This closed list is implemented as a boolean 2D array 
	bool closedList[ROW][COL]; 
	memset(closedList, false, sizeof (closedList)); 

	// Declare a 2D array of structure to hold the details 
	//of that cell 
	// cell cellDetails[ROW][COL]; 
	vector<vector<cell>> cellDetails;

	int i, j; 

	for (i=0; i<ROW; i++) 
	{ 	
		vector<cell> kl;
		for (j=0; j<COL; j++) 
		{ 			
			cell k;
			k.f = FLT_MAX; 
			k.g = FLT_MAX; 
			k.h = FLT_MAX; 
			k.parent_i = -1; 
			k.parent_j = -1; 
			kl.push_back(k);
		} 
		cellDetails.push_back(kl);
	} 

	// Initialising the parameters of the starting node 
	i = src.first, j = src.second; 
	cellDetails[i][j].f = 0.0; 
	cellDetails[i][j].g = 0.0; 
	cellDetails[i][j].h = 0.0; 
	cellDetails[i][j].parent_i = i; 
	cellDetails[i][j].parent_j = j; 

	/* 
	Create an open list having information as- 
	<f, <i, j>> 
	where f = g + h, 
	and i, j are the row and column index of that cell 
	Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1 
	This open list is implenented as a set of pair of pair.*/
	set<pPair> openList; 

	// Put the starting cell on the open list and set its 
	// 'f' as 0 
	openList.insert(make_pair (0.0, make_pair (i, j))); 

	// We set this boolean value as false as initially 
	// the destination is not reached. 
	bool foundDest = false; 

	while (!openList.empty()) 
	{ 
		pPair p = *openList.begin(); 

		// Remove this vertex from the open list 
		openList.erase(openList.begin()); 

		// Add this vertex to the closed list 
		i = p.second.first; 
		j = p.second.second; 
		closedList[i][j] = true; 
	
	/* 
		Generating all the 8 successor of this cell 

			N.W N N.E 
			\ | / 
			\ | / 
			W----Cell----E 
				/ | \ 
			/ | \ 
			S.W S S.E 

		Cell-->Popped Cell (i, j) 
		N --> North	 (i-1, j) 
		S --> South	 (i+1, j) 
		E --> East	 (i, j+1) 
		W --> West		 (i, j-1) 
		N.E--> North-East (i-1, j+1) 
		N.W--> North-West (i-1, j-1) 
		S.E--> South-East (i+1, j+1) 
		S.W--> South-West (i+1, j-1)*/

        const bool driver_axis[3][3] = 
        {
            {0,1,0},
            {1,0,1},
            {0,1,0}
        };

		// To store the 'g', 'h' and 'f' of the 8 successors 
		double gNew, hNew, fNew; 

		//----------- 1st Successor (North) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i-1, j) == true && driver_axis[0][1]) 
		{ 
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i-1, j, dest) == true) 
			{ 
				// Set the Parent of the destination cell 
				cellDetails[i-1][j].parent_i = i; 
				cellDetails[i-1][j].parent_j = j; 
				printf ("The destination cell is found\n"); 
				tracePath (cellDetails, dest, 1); 
				foundDest = true; 
				return; 
			} 
			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i-1][j] == false && 
					isUnBlocked(grid, i-1, j) == true) 
			{ 
				gNew = cellDetails[i][j].g + 1.0; 
				hNew = calculateHValue (i-1, j, dest); 
				fNew = gNew + hNew; 

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//			 OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i-1][j].f == FLT_MAX || 
						cellDetails[i-1][j].f > fNew) 
				{ 
					openList.insert( make_pair(fNew, 
											make_pair(i-1, j))); 

					// Update the details of this cell 
					cellDetails[i-1][j].f = fNew; 
					cellDetails[i-1][j].g = gNew; 
					cellDetails[i-1][j].h = hNew; 
					cellDetails[i-1][j].parent_i = i; 
					cellDetails[i-1][j].parent_j = j; 
				} 
			} 
		} 

		//----------- 2nd Successor (South) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i+1, j) == true && driver_axis[2][1]) 
		{ 
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i+1, j, dest) == true) 
			{ 
				// Set the Parent of the destination cell 
				cellDetails[i+1][j].parent_i = i; 
				cellDetails[i+1][j].parent_j = j; 
				printf("The destination cell is found\n"); 
				tracePath(cellDetails, dest, 2); 
				foundDest = true; 
				return; 
			} 
			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i+1][j] == false && 
					isUnBlocked(grid, i+1, j) == true) 
			{ 
				gNew = cellDetails[i][j].g + 1.0; 
				hNew = calculateHValue(i+1, j, dest); 
				fNew = gNew + hNew; 

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//			 OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i+1][j].f == FLT_MAX || 
						cellDetails[i+1][j].f > fNew) 
				{ 
					openList.insert( make_pair (fNew, make_pair (i+1, j))); 
					// Update the details of this cell 
					cellDetails[i+1][j].f = fNew; 
					cellDetails[i+1][j].g = gNew; 
					cellDetails[i+1][j].h = hNew; 
					cellDetails[i+1][j].parent_i = i; 
					cellDetails[i+1][j].parent_j = j; 
				} 
			} 
		} 

		//----------- 3rd Successor (East) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid (i, j+1) == true) 
		{ 
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, j+1, dest) == true && driver_axis[1][2]) 
			{ 
				// Set the Parent of the destination cell 
				cellDetails[i][j+1].parent_i = i; 
				cellDetails[i][j+1].parent_j = j; 
				printf("The destination cell is found\n"); 
				tracePath(cellDetails, dest, 3); 
				foundDest = true; 
				return; 
			} 

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][j+1] == false && 
					isUnBlocked (grid, i, j+1) == true) 
			{ 
				gNew = cellDetails[i][j].g + 1.0; 
				hNew = calculateHValue (i, j+1, dest); 
				fNew = gNew + hNew; 

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//			 OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][j+1].f == FLT_MAX || 
						cellDetails[i][j+1].f > fNew) 
				{ 
					openList.insert( make_pair(fNew, 
										make_pair (i, j+1))); 

					// Update the details of this cell 
					cellDetails[i][j+1].f = fNew; 
					cellDetails[i][j+1].g = gNew; 
					cellDetails[i][j+1].h = hNew; 
					cellDetails[i][j+1].parent_i = i; 
					cellDetails[i][j+1].parent_j = j; 
				} 
			} 
		} 

		//----------- 4th Successor (West) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j-1) == true && driver_axis[1][0]) 
		{ 
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, j-1, dest) == true) 
			{ 
				// Set the Parent of the destination cell 
				cellDetails[i][j-1].parent_i = i; 
				cellDetails[i][j-1].parent_j = j; 
				printf("The destination cell is found\n"); 
				tracePath(cellDetails, dest, 4); 
				foundDest = true; 
				return; 
			} 

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][j-1] == false && 
					isUnBlocked(grid, i, j-1) == true) 
			{ 
				gNew = cellDetails[i][j].g + 1.0; 
				hNew = calculateHValue(i, j-1, dest); 
				fNew = gNew + hNew; 

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//			 OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][j-1].f == FLT_MAX || 
						cellDetails[i][j-1].f > fNew) 
				{ 
					openList.insert( make_pair (fNew, 
										make_pair (i, j-1))); 

					// Update the details of this cell 
					cellDetails[i][j-1].f = fNew; 
					cellDetails[i][j-1].g = gNew; 
					cellDetails[i][j-1].h = hNew; 
					cellDetails[i][j-1].parent_i = i; 
					cellDetails[i][j-1].parent_j = j; 
				} 
			} 
		} 

		//----------- 5th Successor (North-East) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i-1, j+1) == true && driver_axis[0][2]) 
		{ 
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i-1, j+1, dest) == true) 
			{ 
				// Set the Parent of the destination cell 
				cellDetails[i-1][j+1].parent_i = i; 
				cellDetails[i-1][j+1].parent_j = j; 
				printf ("The destination cell is found\n"); 
				tracePath (cellDetails, dest, 5); 
				foundDest = true; 
				return; 
			} 

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i-1][j+1] == false && 
					isUnBlocked(grid, i-1, j+1) == true) 
			{ 
				gNew = cellDetails[i][j].g + 1.414; 
				hNew = calculateHValue(i-1, j+1, dest); 
				fNew = gNew + hNew; 

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//			 OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i-1][j+1].f == FLT_MAX || 
						cellDetails[i-1][j+1].f > fNew) 
				{ 
					openList.insert( make_pair (fNew, 
									make_pair(i-1, j+1))); 

					// Update the details of this cell 
					cellDetails[i-1][j+1].f = fNew; 
					cellDetails[i-1][j+1].g = gNew; 
					cellDetails[i-1][j+1].h = hNew; 
					cellDetails[i-1][j+1].parent_i = i; 
					cellDetails[i-1][j+1].parent_j = j; 
				} 
			} 
		} 

		//----------- 6th Successor (North-West) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid (i-1, j-1) == true && driver_axis[0][0]) 
		{ 
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination (i-1, j-1, dest) == true) 
			{ 
				// Set the Parent of the destination cell 
				cellDetails[i-1][j-1].parent_i = i; 
				cellDetails[i-1][j-1].parent_j = j; 
				printf ("The destination cell is found\n"); 
				tracePath (cellDetails, dest, 6); 
				foundDest = true; 
				return; 
			} 

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i-1][j-1] == false && 
					isUnBlocked(grid, i-1, j-1) == true) 
			{ 
				gNew = cellDetails[i][j].g + 1.414; 
				hNew = calculateHValue(i-1, j-1, dest); 
				fNew = gNew + hNew; 

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//			 OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i-1][j-1].f == FLT_MAX || 
						cellDetails[i-1][j-1].f > fNew) 
				{ 
					openList.insert( make_pair (fNew, make_pair (i-1, j-1))); 
					// Update the details of this cell 
					cellDetails[i-1][j-1].f = fNew; 
					cellDetails[i-1][j-1].g = gNew; 
					cellDetails[i-1][j-1].h = hNew; 
					cellDetails[i-1][j-1].parent_i = i; 
					cellDetails[i-1][j-1].parent_j = j; 
				} 
			} 
		} 

		//----------- 7th Successor (South-East) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i+1, j+1) == true && driver_axis[2][2]) 
		{ 
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i+1, j+1, dest) == true) 
			{ 
				// Set the Parent of the destination cell 
				cellDetails[i+1][j+1].parent_i = i; 
				cellDetails[i+1][j+1].parent_j = j; 
				printf ("The destination cell is found\n"); 
				tracePath (cellDetails, dest, 7); 
				foundDest = true; 
				return; 
			} 

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i+1][j+1] == false && 
					isUnBlocked(grid, i+1, j+1) == true) 
			{ 
				gNew = cellDetails[i][j].g + 1.414; 
				hNew = calculateHValue(i+1, j+1, dest); 
				fNew = gNew + hNew; 

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//			 OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i+1][j+1].f == FLT_MAX || 
						cellDetails[i+1][j+1].f > fNew) 
				{ 
					openList.insert(make_pair(fNew, 
										make_pair (i+1, j+1))); 

					// Update the details of this cell 
					cellDetails[i+1][j+1].f = fNew; 
					cellDetails[i+1][j+1].g = gNew; 
					cellDetails[i+1][j+1].h = hNew; 
					cellDetails[i+1][j+1].parent_i = i; 
					cellDetails[i+1][j+1].parent_j = j; 
				} 
			} 
		} 

		//----------- 8th Successor (South-West) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid (i+1, j-1) == true && driver_axis[2][0]) 
		{ 
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i+1, j-1, dest) == true) 
			{ 
				// Set the Parent of the destination cell 
				cellDetails[i+1][j-1].parent_i = i; 
				cellDetails[i+1][j-1].parent_j = j; 
				printf("The destination cell is found\n"); 
				tracePath(cellDetails, dest, 8); 
				foundDest = true; 
				return; 
			} 

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i+1][j-1] == false && 
					isUnBlocked(grid, i+1, j-1) == true) 
			{ 
				gNew = cellDetails[i][j].g + 1.414; 
				hNew = calculateHValue(i+1, j-1, dest); 
				fNew = gNew + hNew; 

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//			 OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i+1][j-1].f == FLT_MAX || 
						cellDetails[i+1][j-1].f > fNew) 
				{ 
					openList.insert(make_pair(fNew, 
										make_pair(i+1, j-1))); 

					// Update the details of this cell 
					cellDetails[i+1][j-1].f = fNew; 
					cellDetails[i+1][j-1].g = gNew; 
					cellDetails[i+1][j-1].h = hNew; 
					cellDetails[i+1][j-1].parent_i = i; 
					cellDetails[i+1][j-1].parent_j = j; 
				} 
			} 
		} 
	} 

	// When the destination cell is not found and the open 
	// list is empty, then we conclude that we failed to 
	// reach the destiantion cell. This may happen when the 
	// there is no way to destination cell (due to blockages) 
	if (foundDest == false) 
		printf("Failed to find the Destination Cell\n"); 

	return; 
} 

vector<vector<int32_t>> algoritma_astar::binertoint(String filename){
	vector<vector<int32_t>> XY;
	String txt = "";
	File myFile = SD.open(filename);
	if (myFile) {
		Serial.println(filename);

		// read from the file until there's nothing else in it:
		// String txt = "";
		vector<int32_t> m;
		String txts = "";
		while (myFile.available()) {
			// Serial.write(myFile.read());
			// txt += (char) myFile.read();
			char k = (char) myFile.read();
			if(k == ','){
				m.push_back(txts.toInt());
				txt += String(txts); 
				txt += String(',');
				txts = "";
			} else if (k == '\n'){
				m.push_back(txts.toFloat());
				txt += String(txts); 
				txt += String('\n');
				txts = "";
				XY.push_back(m);
				m.clear();
			} else {
				txts += k;
			}
		}
		myFile.close();
	} else {
		// if the file didn't open, print an error:
		Serial.print("error opening ");
		Serial.println(filename);
		XY.clear();
		return XY;
	}

	// for (auto k : XY){
	// 	for (auto l : k){
	// 		Serial.print(l);
	// 		Serial.print(" ");
	// 	}
	// 	Serial.println();
	// }


    vector<vector<int32_t>> XY_data;
	for (size_t kolom = 0; kolom < XY.size(); kolom++){
		for (size_t baris = 0; baris < XY[kolom].size(); baris++){
			if(XY[kolom][baris] == 1){
				vector<int32_t> vv;
				vv.push_back(baris);
				vv.push_back(kolom);
				XY_data.push_back(vv);
			}
		}
	}
	
	return XY_data;
}

vector<vector<int32_t>> algoritma_astar::Tujuan_koordinat(String filename, String filename_map){
	vector<vector<int32_t>> array_cek;
	if(!manual_dataset){
		vector<vector<float>> data;
		File myFile = SD.open(filename);
		if (myFile) {
			Serial.println(filename);

			// read from the file until there's nothing else in it:
			// String txt = "";
			vector<float> m;
			String txt = "";
			while (myFile.available()) {
				// Serial.write(myFile.read());
				// txt += (char) myFile.read();
				char k = (char) myFile.read();
				if(k == ','){
					m.push_back(txt.toFloat());
					txt = "";
				} else if (k == '\n'){
					m.push_back(txt.toFloat());
					txt = "";
					data.push_back(m);
					m.clear();
				} else {
					txt += k;
				}
			}
			myFile.close();
		} else {
			// if the file didn't open, print an error:
			Serial.print("error opening ");
			Serial.println(filename);
		}

		// for (auto i : data){
		// 	for (auto k : i){
		// 	Serial.print(k);
		// 	Serial.print(" ");
		// 	}    
		// 	Serial.println();
		// }

		vector<vector<uint32_t>> titik;
		vector<vector<int32_t>> XYd = binertoint(filename_map);
		for (int i = 0; i < data.size(); i++){
			float heading = data[i][4];
			vector<uint32_t> m;
			if(data[i][0] > 1 && data[i][1] > 1 && data[i][2] > 1 && data[i][3] > 1){

				uint32_t US_kanan		= data[i][2];
				uint32_t US_kiri 		= data[i][3];
				uint32_t US_belakang 	= data[i][1];
				uint32_t US_depan 		= data[i][0];

				if(heading > (258.0-_US.US_DERAJAT) && heading < (258.0+_US.US_DERAJAT) ){ // 270 derajat
					m.push_back(US_kanan); 		// atas
					m.push_back(US_kiri); 		// bawah
					m.push_back(US_belakang); 	// kanan
					m.push_back(US_depan); 		// kiri
					m.push_back(data[i][5]);
				} else if(heading > (103.0-_US.US_DERAJAT) && heading < (103.0+_US.US_DERAJAT) ){
					m.push_back(US_kiri);
					m.push_back(US_kanan);
					m.push_back(US_depan);
					m.push_back(US_belakang);
					m.push_back(data[i][5]);
				} else if(heading > (188.0-_US.US_DERAJAT) && heading < (188.0+_US.US_DERAJAT) ){
					m.push_back(US_belakang);
					m.push_back(US_depan);
					m.push_back(US_kiri);
					m.push_back(US_kanan);
					m.push_back(data[i][5]);
				} else if(((heading > (351.0-_US.US_DERAJAT) && heading < 360.0) || (heading > 0.0 && heading < ((351.0+_US.US_DERAJAT)-360.0))) ){
					m.push_back(US_depan);
					m.push_back(US_belakang);
					m.push_back(US_kanan);
					m.push_back(US_kiri);
					m.push_back(data[i][5]);
				}
			}
			// for (auto i : m){
			// 	Serial.print(i);
			// 	Serial.print(" ");
			// }
			// Serial.println();		

			vector<int32_t> l;
			if(m[2] < m[3]){
				l.push_back( (X_max - m[2]) / Tinggi_Muka);
			} else {
				l.push_back(m[2] / Tinggi_Muka);
			}   
				
			if(m[0] > m[1]){
				l.push_back( (Y_max - m[1])  / Tinggi_Muka);
			} else {
				l.push_back(m[1] / Tinggi_Muka);
			}	

			l.push_back(m[4]);

			for (auto n : XYd){
				if(l[0] == n[0] && l[1] == n[1]){
					// Serial.print(n[0]);
					// Serial.print(" - ");
					// Serial.println(n[1]);
					array_cek.push_back(l);
				}
			}		
		}


		// if(array_cek.size() == 0){
		// 	array_cek.push_back(l);
		// }
		
		// for (int i = 0; i < array_cek.size(); i++){

		// 	if(l[2] == array_cek[i][2]){
		// 		break;
		// 	}

		// 	// Serial.print(l[0]);
		// 	// Serial.print("-");
		// 	// Serial.print(array_cek[i][0]);
		// 	// Serial.print(" ");
		// 	// Serial.print(l[1]);
		// 	// Serial.print("-");
		// 	// Serial.print(array_cek[i][1]);
		// 	// Serial.print(" ");
		// 	// Serial.print(l[2]);
		// 	// Serial.print("-");
		// 	// Serial.println(array_cek[i][2]);

		// 	if((array_cek.size()-1) == i){
		// 		array_cek.push_back(l);
		// 	}
		// }		

    } else 
	{
		vector<vector<int32_t>> XY;
		String txt = "";
		File myFile = SD.open(filename);
		if (myFile) {
			Serial.println(filename);

			// read from the file until there's nothing else in it:
			// String txt = "";
			vector<int32_t> m;
			String txts = "";
			while (myFile.available()) {
				char k = (char) myFile.read();
				if(k == ','){
					m.push_back(txts.toInt());
					txt += String(txts); 
					txt += String(',');
					txts = "";
				} else if (k == '\n'){
					m.push_back(txts.toFloat());
					txt += String(txts); 
					txt += String('\n');
					txts = "";
					XY.push_back(m);
					m.clear();
				} else {
					txts += k;
				}
			}
			myFile.close();
		} else {
			// if the file didn't open, print an error:
			Serial.print("error opening ");
			Serial.println(filename);
			XY.clear();
			return XY;
		}

		// for (auto k : array_cek){
		// 	for (auto l : k){
		// 		Serial.print(l);
		// 		Serial.print(" ");
		// 	}
		// 	Serial.println();
		// }


		for (size_t kolom = 0; kolom < XY.size(); kolom++){
			for (size_t baris = 0; baris < XY[kolom].size(); baris++){
				if(XY[kolom][baris] > 0){
					vector<int32_t> vv;
					vv.push_back(baris);
					vv.push_back(kolom);
					vv.push_back(XY[kolom][baris] - 1);
					array_cek.push_back(vv);
				}
			}
		}
	}
	
	// array_cek.clear();
	
	// Serial.print(" -==- ");
	// Serial.println(XYd.size());
	return array_cek;
}


