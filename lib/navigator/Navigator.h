#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <stack>
#include <utility>
#include <vector>
#include <map>
#include <memory>
#include <cstring>

#include <webots/Compass.hpp>
#include <Gyroscope.h>

inline constexpr uint8_t DEFAULT_VAL = 128;
inline constexpr int CHUNK_DIM = 256;

// struct Pos
// {
//     int16_t x, y;

//     Pos(int x = -32768, int y = -32768) 
//         : x(static_cast<int16_t>(x)), y(static_cast<int16_t>(y)) {}

//     bool operator<(const Pos &other) const
//     {
//         if (x != other.x)
//             return x < other.x;
//         return y < other.y;
//     }

//     bool operator==(const Pos &other) const
//     {
//         return (x == other.x && y == other.y);
//     }
// };

struct Pos
{
	int16_t x = -32768, y = -32768;

	bool operator<(const Pos &other) const
	{
		if (x != other.x)
			return x < other.x;
		return y < other.y;
	}

	bool operator==(const Pos &other) const
	{
		return (x == other.x && y == other.y);
	}
};

struct Chunk
{
	uint8_t cells[CHUNK_DIM]; // TODO: ogni cella ne continene 2. Doppie celle con la stessa memoria.

	Chunk()
	{
		memset(cells, DEFAULT_VAL, sizeof(cells));
	}
};

struct Route
{
	std::vector<Pos> route;
	double turnAngle = 0;
	int numSteps = -1;
};

struct Node
{
	int parent_i, parent_j;

	int f, g,
		h;
	// g = dalla partenza alla cella attuale, h costo stimato da cella
	// attuale alla destinazione, f è la somma di g e h
};

class Navigator
{
	// matrice virtuale, salvo solo gli ostacoli in coordinate immaginarie
private:
	std::map<Pos, Chunk> _map;

	Pos _currPos = {0, 0};
	Pos _destination;
	double _currDir = 0.0;

	webots::Compass *_compass = nullptr;

	double _compassOffset = 0.0;
	double _currDirCompass = 0.0;
	bool _isSensorsCalibrated = false;

	int _currXWebots;
	int _currYWebots;

	bool isFree(int16_t x, int16_t y);
	Route aStar(Pos start, Pos goal);

	Gyroscope *_gyro = nullptr;

public:
	enum SensorType
	{
		ULTRASONIC = 0,
		LASER = 1,
	};
	static const uint8_t CELL_CM = 10;
	static const uint8_t CHUNK_WIDTH = 16;
	static const uint8_t ULTRASONIC_A = 30;
	static const uint8_t LASER_A = 40;
	static const uint8_t BLANK_A = 20;
	static const uint8_t THRESHOLD_OBSTACLE = DEFAULT_VAL - LASER_A;
	static const uint8_t PADDING_SCULPT = 2;

	Navigator();

	Pos getPos() { return _currPos; }
	Pos calcFinalCell(Pos start, double angle, float dis);
	Route calcRoute(Pos dest);
	int calcDistanceBetween(Pos start, Pos dest);
	static int16_t getPosIndex(Pos p);
	static Pos getChunkPos(Pos p);

	const std::map<Pos, Chunk> &getMap() const;
	double getDir();

	void setDir(double angle);
	void setCurrPos(Pos newPos);
	void createBlanks(Pos target);
	void sculpt(int16_t targetX, int16_t targetY, SensorType st);

	void updateCompass(const double *compassValues);

	void setGyro(Gyroscope *gyro);
};

#endif