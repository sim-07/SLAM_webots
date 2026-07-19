#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <stack>
#include <utility>
#include <vector>
#include <map>
#include <memory>
#include <cstring>

#include <webots/GPS.hpp>

inline constexpr uint8_t DEFAULT_VAL = 128;
inline constexpr int CHUNK_DIM = 256;

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

	Pos _currPos;
	Pos _destination;
	double _currDir = 0.0;

	webots::GPS *_gps = nullptr;

	bool isFree(int16_t x, int16_t y);
	Route aStar(Pos start, Pos goal);

public:
	enum SensorType
	{
		ULTRASONIC = 0,
		LASER = 1,
	};
	static const uint8_t CELL_CM = 10;
	static const uint8_t CHUNK_WIDTH = 16;
	static const uint8_t ULTRASONIC_A = 10;
	static const uint8_t LASER_A = 40;
	static const uint8_t BLANK_A = 40;
	static const uint8_t THRESHOLD_OBSTACLE = DEFAULT_VAL - LASER_A;

	Navigator();

	Pos getPos() { return _currPos; }
	Route calcRoute(int16_t x, int16_t y);
	int calcDistanceBetween(Pos start, Pos dest);
	static int16_t getPosIndex(int16_t x, int16_t y);
	static Pos getChunkPos(int16_t x, int16_t y);

	const std::map<Pos, Chunk> &getMap() const;
	double getDir();

	void setDir(double angle);
	void setCurrPos(int16_t x, int16_t y);
	void createBlanks(int16_t targetX, int16_t targetY);
	void sculpt(int16_t targetX, int16_t targetY, SensorType st);

	void setGps(webots::GPS *gps);
};

#endif