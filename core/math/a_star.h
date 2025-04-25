/**************************************************************************/
/*  a_star.h                                                              */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#ifndef A_STAR_H
#define A_STAR_H

#include <unordered_map>

#include "core/object/gdvirtual.gen.inc"
#include "core/object/ref_counted.h"
#include "core/templates/oa_hash_map.h"

/**
	A* pathfinding algorithm.
*/

using PointID = int32_t;

class AStar3D : public RefCounted {
	GDCLASS(AStar3D, RefCounted);
	friend class AStar2D;

	struct Point {
		Point() {}

		PointID id = 0;
		Vector3 pos;
		real_t weight_scale = 0;
		bool enabled = false;
		int64_t flags = 0;

		OAHashMap<PointID, Point *> neighbors = 4u;
		OAHashMap<PointID, Point *> unlinked_neighbours = 4u;

		// Used for pathfinding.
		Point *prev_point = nullptr;
		real_t g_score = 0;
		real_t f_score = 0;
		uint64_t open_pass = 0;
		uint64_t closed_pass = 0;
	};

	struct SortPoints {
		_FORCE_INLINE_ bool operator()(const Point *A, const Point *B) const { // Returns true when the Point A is worse than Point B.
			if (A->f_score > B->f_score) {
				return true;
			} else if (A->f_score < B->f_score) {
				return false;
			} else {
				return A->g_score < B->g_score; // If the f_costs are the same then prioritize the points that are further away from the start.
			}
		}
	};

	struct Segment {
		Pair<PointID, PointID> key;

		enum {
			NONE = 0,
			FORWARD = 1,
			BACKWARD = 2,
			BIDIRECTIONAL = FORWARD | BACKWARD
		};
		unsigned char direction = NONE;

		static uint32_t hash(const Segment &p_seg) {
			return PairHash<PointID, PointID>().hash(p_seg.key);
		}
		bool operator==(const Segment &p_s) const { return key == p_s.key; }

		Segment() {}
		Segment(PointID p_from, PointID p_to) {
			if (p_from < p_to) {
				key.first = p_from;
				key.second = p_to;
				direction = FORWARD;
			} else {
				key.first = p_to;
				key.second = p_from;
				direction = BACKWARD;
			}
		}
	};

	PointID last_free_id = 0;
	uint64_t pass = 1;

	OAHashMap<PointID, Point *> points;
	HashSet<Segment, Segment> segments;
	OAHashMap<Segment, int64_t, Segment> segment_flags = 4u;

	bool _solve(Point *begin_point, Point *end_point, int64_t required_flags = 0, int64_t skip_flags = 0, int64_t connection_skip_flags = 0);

protected:
	static void _bind_methods();

	virtual real_t _estimate_cost(PointID p_from_id, PointID p_to_id);
	virtual real_t _compute_cost(PointID p_from_id, PointID p_to_id);

	GDVIRTUAL2RC(real_t, _estimate_cost, PointID, PointID)
	GDVIRTUAL2RC(real_t, _compute_cost, PointID, PointID)

public:
	PointID get_available_point_id() const;

	void add_point(PointID p_id, const Vector3 &p_pos, real_t p_weight_scale = 1);
	Vector3 get_point_position(PointID p_id) const;
	void set_point_position(PointID p_id, const Vector3 &p_pos);
	real_t get_point_weight_scale(PointID p_id) const;
	void set_point_weight_scale(PointID p_id, real_t p_weight_scale);
	void remove_point(PointID p_id);
	bool has_point(PointID p_id) const;
	Vector<PointID> get_point_connections(PointID p_id);
	PackedInt32Array get_point_ids();
	Vector<PointID> get_connections();

	void set_point_disabled(PointID p_id, bool p_disabled = true);
	bool is_point_disabled(PointID p_id) const;

	void connect_points(PointID p_id, PointID p_with_id, bool bidirectional = true);
	void disconnect_points(PointID p_id, PointID p_with_id, bool bidirectional = true);
	bool are_points_connected(PointID p_id, PointID p_with_id, bool bidirectional = true) const;

	int64_t get_point_count() const;
	int64_t get_point_capacity() const;
	void reserve_space(int64_t p_num_nodes);
	void clear();

	PointID get_closest_point(const Vector3 &p_point, bool p_include_disabled = false) const;
	Vector3 get_closest_position_in_segment(const Vector3 &p_point) const;

	Vector<Vector3> get_point_path(PointID p_from_id, PointID p_to_id, int64_t required_flags = 0, int64_t skip_flags = 0);
	Vector<PointID> get_id_path(PointID p_from_id, PointID p_to_id, int64_t required_flags = 0, int64_t skip_flags = 0, int64_t connection_skip_flags = 0);

	void add_flags(PointID p_id, int64_t flag);
	void remove_flags(PointID p_id, int64_t flag);
	void set_flags(PointID p_id, int64_t flag);
	bool has_flags(PointID p_id, int64_t flag);
	void clear_flags(PointID p_id);
	int64_t get_flags(PointID p_id) const;

	void add_connection_flags(PointID a, PointID b, int64_t flag);
	void remove_connection_flags(PointID a, PointID b, int64_t flag);
	int64_t get_connection_flags(PointID a, PointID b);

	AStar3D() {}
	~AStar3D();
};

class AStar2D : public RefCounted {
	GDCLASS(AStar2D, RefCounted);
	AStar3D astar;

	bool _solve(AStar3D::Point *begin_point, AStar3D::Point *end_point);

protected:
	static void _bind_methods();

	virtual real_t _estimate_cost(int64_t p_from_id, int64_t p_to_id);
	virtual real_t _compute_cost(int64_t p_from_id, int64_t p_to_id);

	GDVIRTUAL2RC(real_t, _estimate_cost, int64_t, int64_t)
	GDVIRTUAL2RC(real_t, _compute_cost, int64_t, int64_t)

public:
	int64_t get_available_point_id() const;

	void add_point(int64_t p_id, const Vector2 &p_pos, real_t p_weight_scale = 1);
	Vector2 get_point_position(int64_t p_id) const;
	void set_point_position(int64_t p_id, const Vector2 &p_pos);
	real_t get_point_weight_scale(int64_t p_id) const;
	void set_point_weight_scale(int64_t p_id, real_t p_weight_scale);
	void remove_point(int64_t p_id);
	bool has_point(int64_t p_id) const;
	Vector<PointID> get_point_connections(PointID p_id);
	PackedInt32Array get_point_ids();

	void set_point_disabled(int64_t p_id, bool p_disabled = true);
	bool is_point_disabled(int64_t p_id) const;

	void connect_points(int64_t p_id, int64_t p_with_id, bool p_bidirectional = true);
	void disconnect_points(int64_t p_id, int64_t p_with_id, bool p_bidirectional = true);
	bool are_points_connected(int64_t p_id, int64_t p_with_id, bool p_bidirectional = true) const;

	int64_t get_point_count() const;
	int64_t get_point_capacity() const;
	void reserve_space(int64_t p_num_nodes);
	void clear();

	int64_t get_closest_point(const Vector2 &p_point, bool p_include_disabled = false) const;
	Vector2 get_closest_position_in_segment(const Vector2 &p_point) const;

	Vector<Vector2> get_point_path(int64_t p_from_id, int64_t p_to_id);
	Vector<int64_t> get_id_path(int64_t p_from_id, int64_t p_to_id);

	AStar2D() {}
	~AStar2D() {}
};

#endif // A_STAR_H
