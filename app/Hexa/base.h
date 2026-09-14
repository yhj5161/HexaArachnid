#ifndef __HEXA_BASE_H
#define __HEXA_BASE_H

typedef struct {
	float x, y, z;
} Point3D;

/* Locations = 6 足端坐标组成的帧（Point3D[6]，兼容 pathTool 生成格式） */
typedef Point3D Locations[6];

/* 向量运算 */
static inline Point3D p3d_sub(Point3D a, Point3D b) {
	Point3D r = { a.x - b.x, a.y - b.y, a.z - b.z };
	return r;
}
static inline Point3D p3d_mul(Point3D a, float b) {
	Point3D r = { a.x * b, a.y * b, a.z * b };
	return r;
}
static inline void p3d_addEq(Point3D *a, Point3D b) {
	a->x += b.x; a->y += b.y; a->z += b.z;
}
static inline int p3d_eq(Point3D a, Point3D b) {
	return (a.x == b.x && a.y == b.y && a.z == b.z);
}

/* Locations 运算（Locations = Point3D[6]，用指针避免数组值传递） */
static inline void loc_sub(const Locations *a, const Locations *b, Locations *result) {
	int i;
	for (i = 0; i < 6; i++) (*result)[i] = p3d_sub((*a)[i], (*b)[i]);
}
static inline void loc_mul(const Locations *a, float b, Locations *result) {
	int i;
	for (i = 0; i < 6; i++) (*result)[i] = p3d_mul((*a)[i], b);
}
static inline void loc_addEq(Locations *a, const Locations *b) {
	int i;
	for (i = 0; i < 6; i++) p3d_addEq(&(*a)[i], (*b)[i]);
}

#endif