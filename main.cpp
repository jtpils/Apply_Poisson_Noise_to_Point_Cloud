// 点群にポアソンノイズを適用するプログラム
// local/KVS/Makefile.def:312

// ./applyPoissonNoise [diff] [ratio_for_lamda] [ratio_for_apply_noise]

// 実行例
// ./applyPoissonNoise 0.01 0.0001 0.2

#include <kvs/MersenneTwister>
#include <kvs/Vector3>
#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <random>	// for poisson distribution

// default output file name
const char OUTPUT_SPBR[]		= "box";
const char OUTPUT_NOISED_SPBR[]	= "box_noised";

int main(int argc, char **argv) {
	if ( argc != 4 ) {
		std::cout << "USAGE: " << argv[0] << " [diff] [ratio_for_lamda] [ratio_for_apply_noise]" << std::endl;
		exit(1);
	}

	std::random_device 	rd;
    std::mt19937 		gen(rd());


	// ----------------
	// ----- FILE -----
	// ----------------
	// raw point cloud
	std::ofstream 	fout_raw;
	std::string		of_name_raw( OUTPUT_SPBR );
	of_name_raw		+= "_";
	of_name_raw		+= argv[1];
	of_name_raw		+= ".spbr";
	fout_raw.open( of_name_raw );

	// noised point cloud
	std::ofstream	fout_noised;
	std::string		of_name_noised( OUTPUT_NOISED_SPBR );
	of_name_noised	+= "_";
	of_name_noised	+= argv[2];
	of_name_noised	+= "_";
	of_name_noised	+= argv[3];
	of_name_noised	+= ".spbr";
	fout_noised.open( of_name_noised );



	// ------------------------------------
	// ----- Generate raw point cloud -----
	// ------------------------------------
	// local variables
	std::vector<kvs::Vector3d> 	box_points;
	kvs::Vector3d 				box_point;
	float diff = atof(argv[1]);

	// header for spbr
	fout_raw << "#/SPBR_ASCII_Data"			<< std::endl;
	fout_raw << "#/RepeatLevel 1"			<< std::endl;
	fout_raw << "#/BGColorRGBByte 0 0 0" 	<< std::endl;
	fout_raw << "#/ColorRGB 255 255 255"	<< std::endl;
	fout_raw << "#/ImageResolution 1440"	<< std::endl;
	fout_raw << "#/LOD 0"					<< std::endl;
	fout_raw << "#/EndHeader"				<< std::endl;

	// generate box point cloud
	for (float diff_z = 0.0; diff_z < 1.0; diff_z += diff) {
		for (float diff_y = 0.0; diff_y < 1.0; diff_y += diff) {
			for (float diff_x = 0.0; diff_x < 1.0; diff_x += diff) {
				box_point.set(diff_x, diff_y, diff_z);
				box_points.push_back( box_point );

				// write to spbr file
				fout_raw << box_point << std::endl;
			}
		}
	}

	// standard output
	float ratio_for_lamda = atof(argv[2]);
	float diagonal_length = sqrtf(1.0*1.0 + 1.0*1.0 + 1.0*1.0);
	float lamda = diagonal_length * ratio_for_lamda;
	std::cout << "Diagonal length"					<< std::endl;
	std::cout << "> " << diagonal_length 			<< std::endl;
	std::cout << "\nLamda(average)" 				<< std::endl;
	std::cout << "> " << lamda						<< std::endl;
	std::cout << "\nNumber of points \"box.spbr\""	<< std::endl;    
    std::cout << "> " << box_points.size() << "\n" 	<< std::endl;



    // --------------------------------------------------
    // ----- Apply Poisson noise to box point cloud -----
    // --------------------------------------------------
    // local variables
    kvs::MersenneTwister			uniRand;
    std::poisson_distribution<> 	poissonRand(lamda);
	kvs::Vector3d 					point;
	float ratio_for_apply_noise = atof(argv[3]);
	float x, y, z;
	int noise_counter = 0;

	// header for spbr
    fout_noised << "#/SPBR_ASCII_Data"			<< std::endl;
	fout_noised << "#/RepeatLevel 1"			<< std::endl;
	fout_noised << "#/BGColorRGBByte 0 0 0" 	<< std::endl;
	fout_noised << "#/ColorRGB 255 0 0"			<< std::endl;
	fout_noised << "#/ImageResolution 1440"		<< std::endl;
	fout_noised << "#/LOD 0"					<< std::endl;
	fout_noised << "#/EndHeader"				<< std::endl;

	// stochastically apply poisson noise to box point cloud
	for (int i = 0; i < box_points.size(); i++) {
		if ( uniRand() < ratio_for_apply_noise ) {
			// P(λ)
			x = box_points[i].x() + poissonRand(gen);
			y = box_points[i].y() + poissonRand(gen);
			z = box_points[i].z() + poissonRand(gen);
			// x = poissonRand(gen);
			// y = poissonRand(gen);
			// z = poissonRand(gen);
			point.set(x, y, z);

			// write to spbr file
			fout_noised << point << std::endl;

			noise_counter++;
		}
	}
	std::cout << "\nNumber of noised points"	<< std::endl;    
    std::cout << "> " << noise_counter << "\n" 	<< std::endl; 



	// ----------------------
	// ----- Exec. SPBR -----
	// ----------------------
	std::string EXEC("./spbr ");
	EXEC += of_name_raw;
	EXEC += " ";
	EXEC += of_name_noised;
	system( EXEC.c_str() );



	// ----------------------
	// ----- FILE close -----
	// ----------------------
	fout_raw.close();
	fout_noised.close();



	return 0;
}