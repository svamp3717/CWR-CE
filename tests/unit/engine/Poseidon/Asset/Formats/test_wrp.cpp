#include <catch2/catch_test_macros.hpp>
#include <Poseidon/IO/Streams/QStream.hpp>
#include <Poseidon/IO/Streams/SerializeBin.hpp>
#include <Poseidon/IO/Serialization/SerializeBinExt.hpp>
#include <Poseidon/World/Terrain/WrpReader.hpp>
#include "test_fixtures.hpp"
#include <stdio.h>
#include <catch2/catch_message.hpp>
#include <string>
#include <vector>
#include <Poseidon/Foundation/platform.hpp>

using namespace Poseidon;

#ifdef GetObject
#undef GetObject
#endif

TEST_CASE("WRP: WrpReader loads sparse object IDs from fixture terrain", "[Formats][WRP]")
{
    const char* path = GET_FIXTURE("wrp/test_world.wrp");

    WrpReader reader;
    REQUIRE(reader.Load(path));
    REQUIRE(reader.GetFormat() == WrpReader::RVW_V4);
    REQUIRE(reader.GetGridX() == 4);
    REQUIRE(reader.GetGridZ() == 4);
    REQUIRE(reader.GetHeightmapSize() == 16);
    REQUIRE(reader.GetTextureCount() == 1);
    REQUIRE(reader.GetObjectCount() == 3);
    CHECK(reader.GetObject(0).id == 17);
    CHECK(reader.GetObject(1).id == 2);
    CHECK(reader.GetObject(2).id == 9);
    CHECK(std::string(reader.GetObject(0).name) == R"(data3d\dummy17.p3d)");
    CHECK(std::string(reader.GetObject(1).name) == R"(data3d\dummy2.p3d)");
    CHECK(std::string(reader.GetObject(2).name) == R"(data3d\dummy9.p3d)");
}

TEST_CASE("WRP: WrpReader loads large OPRW with objects", "[Formats][WRP][GameData]")
{
    // Game data lives under GAME_DATA_DIR/Worlds/.
    char abelPath[MAX_PATH];
    snprintf(abelPath, sizeof(abelPath), "%s/Worlds/abel.wrp", GAME_DATA_DIR);

    QIFStream probe;
    probe.open(abelPath);
    if (probe.fail())
    {
        SKIP("Game world abel.wrp not available");
        return;
    }

    WrpReader reader;
    REQUIRE(reader.Load(abelPath));
    REQUIRE((reader.GetFormat() == WrpReader::OPRW_V2 || reader.GetFormat() == WrpReader::OPRW_V3));
    REQUIRE(reader.GetObjectCount() > 0);
    REQUIRE(reader.GetObjectNameCount() > 0);
    REQUIRE(reader.GetTextureCount() > 0);

    // Verify first object has valid data
    const auto& obj = reader.GetObject(0);
    REQUIRE(obj.name.GetLength() > 0);
    REQUIRE(obj.hasMatrix == true);
}

TEST_CASE("WRP: WrpReader reports error for missing file", "[Formats][WRP]")
{
    WrpReader reader;
    REQUIRE_FALSE(reader.Load("nonexistent.wrp"));
    REQUIRE(reader.GetError() != nullptr);
}

TEST_CASE("WRP: WrpReader reports error for invalid data", "[Formats][WRP]")
{
    // Create a local temp file with invalid magic. Keep this relative so it
    // works under Windows ctest runs without relying on /tmp.
    const char* tmpPath = "test_invalid.wrp";
    {
        QOFStream f;
        f.open(tmpPath);
        int badMagic = 0xDEADBEEF;
        f.write(&badMagic, sizeof(badMagic));
        f.close();
    }

    WrpReader reader;
    REQUIRE_FALSE(reader.Load(tmpPath));
    REQUIRE(std::string(reader.GetError()) == "Unknown file format");

    remove(tmpPath);
}

static std::vector<char> BuildRvw4Blob()
{
    std::vector<char> blob;
    auto put = [&blob](const void* p, size_t n)
    {
        const char* c = static_cast<const char*>(p);
        blob.insert(blob.end(), c, c + n);
    };

    put("4WVR", 4);
    int range = 4;
    put(&range, 4); // xRange
    put(&range, 4); // zRange

    short cell = 0;
    for (int i = 0; i < 4 * 4; i++)
        put(&cell, sizeof(cell)); // heightmap
    for (int i = 0; i < 4 * 4; i++)
        put(&cell, sizeof(cell)); // texture indices

    char texName[32] = R"(landtext\mo.pac)";
    put(texName, sizeof(texName));
    char emptyTex[32] = {};
    for (int i = 1; i < 512; i++)
        put(emptyTex, sizeof(emptyTex));

    for (int id = 0; id < 2; id++)
    {
        float matrix[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 100.0f + id, 0, 200.0f};
        put(matrix, sizeof(matrix));
        put(&id, sizeof(id));
        char name[76] = {};
        snprintf(name, sizeof(name), R"(data3d\dummy%d.p3d)", id);
        put(name, sizeof(name));
    }
    return blob;
}

TEST_CASE("WRP: RVW re-parse after a probe-to-EOF and seekg sees all objects", "[Formats][WRP]")
{
    std::vector<char> blob = BuildRvw4Blob();

    QIStream s(blob.data(), (int)blob.size());

    WrpReader probe;
    REQUIRE(probe.Load(s));
    REQUIRE(probe.GetFormat() == WrpReader::RVW_V4);
    REQUIRE(probe.GetObjectCount() == 2);

    s.seekg(0, QIOS::beg);
    REQUIRE_FALSE(s.fail());
    REQUIRE_FALSE(s.eof());

    WrpReader fallback;
    REQUIRE(fallback.Load(s));
    REQUIRE(fallback.GetFormat() == WrpReader::RVW_V4);
    REQUIRE(fallback.GetObjectCount() == 2);
    REQUIRE(std::string((const char*)fallback.GetObject(0).name) == R"(data3d\dummy0.p3d)");
}

namespace
{
std::vector<char> BuildOprw3Blob()
{
    QOStream out;
    SerializeBinStream sf(&out);
    out.write("OPRW", 4);
    sf.SaveInt(3);
    for (int i = 0; i < 4; ++i)
        sf.SaveInt(4); // square grid and matching heightmap dimensions

    // 4x4 cells; these small arrays are uncompressed in OPRW.
    char geography[4 * 4 * 4] = {};
    char sound[4 * 4] = {};
    short textureIndices[4 * 4] = {};
    char random[4 * 4 * 4] = {};
    float heights[4 * 4] = {};
    sf.SaveCompressed(geography, sizeof(geography));
    sf.SaveCompressed(sound, sizeof(sound));
    sf.SaveInt(0); // no mountain peaks
    sf.SaveCompressed(textureIndices, sizeof(textureIndices));
    sf.SaveCompressed(random, sizeof(random));
    sf.SaveCompressed(heights, sizeof(heights));

    sf.SaveInt(1); // texture table
    RStringB textureName = R"(landtext\mo.pac)";
    sf.Transfer(textureName);
    bool enableUV = false;
    sf.Transfer(enableUV);

    sf.SaveInt(1); // object name table
    RStringB objectName = R"(data3d\dummy.p3d)";
    sf.Transfer(objectName);

    Matrix4 transform(MScale, 1.0f); // valid identity transform, serialized as Matrix4P
    for (int id : {7, 9})
    {
        sf.SaveInt(id);
        sf.SaveInt(0); // index into object name table
        sf.Transfer(transform);
    }
    sf.SaveInt(-1); // object terminator

    return std::vector<char>(out.str(), out.str() + out.pcount());
}
} // namespace

TEST_CASE("WRP: OPRW can read object records incrementally", "[Formats][WRP]")
{
    const auto blob = BuildOprw3Blob();

    QIStream eagerStream(blob.data(), static_cast<int>(blob.size()));
    WrpReader eager;
    REQUIRE(eager.Load(eagerStream));
    REQUIRE(eager.GetObjectCount() == 2);

    QIStream stream(blob.data(), static_cast<int>(blob.size()));
    WrpReader reader;
    REQUIRE(reader.Load(stream, true));
    REQUIRE(reader.GetFormat() == WrpReader::OPRW_V3);
    REQUIRE(reader.GetHeightmapSize() == 16);
    REQUIRE(reader.GetObjectCount() == 0); // no duplicated in-memory object list
    WrpObjectInfo object;
    REQUIRE(reader.ReadNextOPRWObject(stream, object) == WrpReader::ObjectReadResult::Object);
    CHECK(object.id == 7);
    CHECK(std::string(object.name) == R"(data3d\dummy.p3d)");
    REQUIRE(reader.ReadNextOPRWObject(stream, object) == WrpReader::ObjectReadResult::Object);
    CHECK(object.id == 9);
    REQUIRE(reader.ReadNextOPRWObject(stream, object) == WrpReader::ObjectReadResult::End);
}

TEST_CASE("WRP: truncated deferred OPRW object fails rather than partially loading", "[Formats][WRP]")
{
    auto blob = BuildOprw3Blob();
    blob.resize(blob.size() - sizeof(Matrix4P) - sizeof(int)); // remove final transform and terminator

    QIStream stream(blob.data(), static_cast<int>(blob.size()));
    WrpReader reader;
    REQUIRE(reader.Load(stream, true));

    WrpObjectInfo object;
    REQUIRE(reader.ReadNextOPRWObject(stream, object) == WrpReader::ObjectReadResult::Object);
    REQUIRE(reader.ReadNextOPRWObject(stream, object) == WrpReader::ObjectReadResult::Error);
    REQUIRE(reader.GetError() != nullptr);
}
