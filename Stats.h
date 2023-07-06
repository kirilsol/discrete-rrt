#include <iostream>
#include <fstream>
#include <string>

using namespace std;

class Stats
{
public:

	Stats()
	{

		// force execute if the program is ran by a script
		std::ifstream ifile0("nogui.txt");
		std::istream *in0 = &ifile0; 
		bool flag_no_gui;
		*in0 >> flag_no_gui;

		if (flag_no_gui)
		{
			string dummy;
			
			std::ifstream ifile1("Stats\\is_basic.txt");
			std::istream *in1 = &ifile1; 

			*in1 >> IS_BASIC;

			std::ifstream ifile2("Stats\\basic\\attempts_threshold.txt");
			std::istream *in2 = &ifile2; 
			*in2 >> basic_CONFSET_ATTEMPTS_TRESHOLD;

			std::ifstream ifile3("Stats\\basic\\extconfs.txt");
			std::istream *in3 = &ifile3; 
			*in3 >> basic_EXTCONFS;

			std::ifstream ifile4("Stats\\basic\\pair_connections.txt");
			std::istream *in4 = &ifile4; 
			*in4 >> basic_PAIR_EXT_CONNECTIONS;

			std::ifstream ifile5("Stats\\basic\\start_target_connetions.txt");
			std::istream *in5 = &ifile5; 
			*in5 >> basic_START_TARGET_CONNECTIONS;

			std::ifstream ifile6("Stats\\complex\\conf_size.txt");
			std::istream *in6 = &ifile6; 
			*in6 >> complex_MAX_CONFSET_SIZE;

			std::ifstream ifile7("Stats\\complex\\attempts_threshold.txt");
			std::istream *in7 = &ifile7; 
			*in7 >> complex_CONFSET_ATTEMPTS_TRESHOLD;
			
			std::ifstream ifile8("Stats\\complex\\extconfs.txt");
			std::istream *in8 = &ifile8; 
			*in8 >> complex_EXTCONFS;

			std::ifstream ifile9("Stats\\complex\\pair_connections.txt");
			std::istream *in9 = &ifile9; 
			*in9 >> complex_PAIR_EXT_CONNECTIONS;

			std::ifstream ifile10("Stats\\complex\\start_target_connetions.txt");
			std::istream *in10 = &ifile10; 
			*in10 >> complex_START_TARGET_CONNECTIONS;

			std::ifstream ifile11("Stats\\basic\\hausdorff.txt");
			std::istream *in11 = &ifile11; 
			*in11 >> HAUSDORFF_CLOSEST;

			ifile1.close();
			ifile2.close();
			ifile3.close();
			ifile4.close();
			ifile5.close();
			ifile6.close();
			ifile7.close();
			ifile8.close();
			ifile9.close();
			ifile10.close();
			ifile11.close();
		}
		else
		{
			std::ifstream ifile("stats.txt");
			std::istream *in = &ifile; 

			string dummy;

			*in >> dummy >> IS_BASIC;
			*in >> dummy >> IS_SVESTKA;

			*in >> dummy >> basic_CONFSET_ATTEMPTS_TRESHOLD;
			*in >> dummy >> basic_EXTCONFS;
			*in >> dummy >> basic_PAIR_EXT_CONNECTIONS;
			*in >> dummy >> basic_START_TARGET_CONNECTIONS;

			*in >> dummy >> complex_MAX_CONFSET_SIZE;
			*in >> dummy >> complex_CONFSET_ATTEMPTS_TRESHOLD;
			*in >> dummy >> complex_EXTCONFS;
			*in >> dummy >> complex_PAIR_EXT_CONNECTIONS;
			*in >> dummy >> complex_START_TARGET_CONNECTIONS;

			*in >> dummy >> HAUSDORFF_CLOSEST;

			*in >> dummy >> svestka_NUM_VERTICES;
			*in >> dummy >> svestka_KNEAREST;

			ifile.close();
		}

		ifile0.close();


		
	}

	~Stats(){}

	bool	IS_BASIC;
	bool	IS_SVESTKA;
	
	int		basic_CONFSET_ATTEMPTS_TRESHOLD;
	int		basic_EXTCONFS;
	int		basic_PAIR_EXT_CONNECTIONS;
	int		basic_START_TARGET_CONNECTIONS;

	int		complex_MAX_CONFSET_SIZE;
	int		complex_CONFSET_ATTEMPTS_TRESHOLD;
	int		complex_EXTCONFS;
	int		complex_PAIR_EXT_CONNECTIONS;
	int		complex_START_TARGET_CONNECTIONS;

	int		HAUSDORFF_CLOSEST;

	int		svestka_NUM_VERTICES;
	int		svestka_KNEAREST;
};
