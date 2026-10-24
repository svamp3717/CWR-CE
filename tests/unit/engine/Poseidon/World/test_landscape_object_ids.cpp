#include <catch2/catch_test_macros.hpp>
#include <Poseidon/World/Scene/Object.hpp>
#include <Poseidon/World/Terrain/Landscape.hpp>
#include <Poseidon/World/Terrain/WrpReader.hpp>
#include "test_fixtures.hpp"

using namespace Poseidon;

#ifdef GetObject
#undef GetObject
#endif

namespace
{
class GlobalLandscapeScope
{
  public:
    explicit GlobalLandscapeScope(Landscape* landscape) : _previous(GLandscape) { GLandscape = landscape; }

    ~GlobalLandscapeScope() { GLandscape = _previous; }

  private:
    Landscape* _previous;
};

Ref<ObjectPlain> AddLandscapeObject(Landscape& landscape, int id)
{
    Ref<ObjectPlain> object = new ObjectPlain(nullptr, id);
    landscape.AddObject(object, nullptr, nullptr, true);
    return object;
}
} // namespace

TEST_CASE("Landscape ID cache uses the highest sparse object ID", "[World][Terrain][ObjectID]")
{
    WrpReader reader;
    REQUIRE(reader.Load(GET_FIXTURE("wrp/test_world.wrp")));

    Landscape landscape(nullptr, nullptr);
    GlobalLandscapeScope globalLandscape(&landscape);
    Ref<ObjectPlain> highObject = AddLandscapeObject(landscape, reader.GetObject(0).id);
    Ref<ObjectPlain> lowObject = AddLandscapeObject(landscape, reader.GetObject(1).id);
    Ref<ObjectPlain> middleObject = AddLandscapeObject(landscape, reader.GetObject(2).id);

    landscape.RebuildIDCache();

    REQUIRE(landscape.GetLastObjectID() == 17);
    REQUIRE(landscape.GetObject(2) == lowObject);
    REQUIRE(landscape.GetObject(9) == middleObject);
    REQUIRE(landscape.GetObject(17) == highObject);
}

TEST_CASE("Landscape runtime IDs preserve terrain cache entries", "[World][Terrain][ObjectID]")
{
    Landscape landscape(nullptr, nullptr);
    GlobalLandscapeScope globalLandscape(&landscape);
    Ref<ObjectPlain> terrainObject = AddLandscapeObject(landscape, 2);
    Ref<ObjectPlain> highTerrainObject = AddLandscapeObject(landscape, 17);

    landscape.RebuildIDCache();

    Ref<ObjectPlain> firstRuntimeObject = new ObjectPlain(nullptr, landscape.NewObjectID());
    Ref<ObjectPlain> secondRuntimeObject = new ObjectPlain(nullptr, landscape.NewObjectID());
    Ref<ObjectPlain> thirdRuntimeObject = new ObjectPlain(nullptr, landscape.NewObjectID());
    landscape.AddToIDCache(firstRuntimeObject);
    landscape.AddToIDCache(secondRuntimeObject);
    landscape.AddToIDCache(thirdRuntimeObject);

    CHECK(firstRuntimeObject->ID() == 18);
    CHECK(secondRuntimeObject->ID() == 19);
    CHECK(thirdRuntimeObject->ID() == 20);
    CHECK(landscape.GetObject(18) == firstRuntimeObject);
    CHECK(landscape.GetObject(19) == secondRuntimeObject);
    CHECK(landscape.GetObject(20) == thirdRuntimeObject);
    REQUIRE(landscape.GetObject(2) == terrainObject);
    REQUIRE(landscape.GetObject(17) == highTerrainObject);

    landscape.GetObject(2)->SetDammage(0.5f);
    landscape.GetObject(17)->SetDammage(0.25f);
    REQUIRE(terrainObject->GetTotalDammage() == 0.5f);
    REQUIRE(highTerrainObject->GetTotalDammage() == 0.25f);
}

TEST_CASE("Landscape ID cache rebuild raises the runtime ID floor", "[World][Terrain][ObjectID]")
{
    Landscape landscape(nullptr, nullptr);
    GlobalLandscapeScope globalLandscape(&landscape);
    Ref<ObjectPlain> terrainObject = AddLandscapeObject(landscape, 2);

    landscape.RebuildIDCache();
    REQUIRE(landscape.GetLastObjectID() == 2);

    Ref<ObjectPlain> runtimeObject = AddLandscapeObject(landscape, landscape.NewObjectID());
    Ref<ObjectPlain> laterTerrainObject = AddLandscapeObject(landscape, 42);
    landscape.RebuildIDCache();

    REQUIRE(landscape.GetLastObjectID() == 42);
    REQUIRE(landscape.GetObject(2) == terrainObject);
    REQUIRE(landscape.GetObject(3) == runtimeObject);
    REQUIRE(landscape.GetObject(42) == laterTerrainObject);
    REQUIRE(landscape.NewObjectID() == 43);
}

TEST_CASE("Bulk WRP object insertion expands cell bounds without rescanning", "[World][Terrain][WRP][Performance]")
{
    // The old per-object StaticChanged() rescanned the whole cell after
    // every inserted tree/rock/building: O(n^2) for a dense 750 MB island.
    // The incremental bound must still contain every inserted static object.
    ObjectListFull list(0, 0);
    list.SetBSphere(VZero, 50);

    for (int i = 0; i < 400; ++i)
    {
        Ref<ObjectPlain> object = new ObjectPlain(nullptr, i);
        object->SetPosition(Vector3(static_cast<float>(i), 0, 0));
        REQUIRE(list.Add(object, true) >= 0);
    }

    REQUIRE(list.Size() == 400);
    CHECK(list.GetNonStaticCount() == 0);
    CHECK(list.GetBSphereRadius() >= 399.0f);

    // Rebuilding after the load may tighten but must not enlarge the bound.
    const float incrementalRadius = list.GetBSphereRadius();
    list.StaticChanged();
    CHECK(list.GetBSphereRadius() <= incrementalRadius);
    CHECK(list.GetBSphereRadius() >= 399.0f);
}
