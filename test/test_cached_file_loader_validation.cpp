// [camp#177] Image-validation gate for the shared tile/file cache: WMS servers
// report errors as HTTP 200 + XML ServiceExceptionReport, which must never be
// cached as tile bodies. Covers CachedFileLoader::isAcceptableImageBody (the
// Content-Type fast reject + QImage decode authority) and the caller-awareness
// contract on CachedFileClient (tile loaders opt in; capabilities XML clients
// keep the ungated default). No network.

#include <gtest/gtest.h>

#include <QBuffer>
#include <QByteArray>
#include <QImage>
#include <QString>

#include "util/cached_file_loader.h"

using camp::CachedFileClient;
using camp::CachedFileLoader;

namespace
{

// A representative WMS 1.1.1 error body: what GEBCO returns with HTTP 200
// when the backend hiccups.
QByteArray serviceExceptionXml()
{
  return QByteArray(
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
    "<ServiceExceptionReport version=\"1.1.1\">"
    "<ServiceException>Internal error</ServiceException>"
    "</ServiceExceptionReport>");
}

// A minimal valid PNG, built in-memory so the test carries no binary fixture.
QByteArray validPngBytes()
{
  QImage image(2, 2, QImage::Format_RGB32);
  image.fill(Qt::blue);
  QByteArray bytes;
  QBuffer buffer(&bytes);
  EXPECT_TRUE(buffer.open(QIODevice::WriteOnly));
  EXPECT_TRUE(image.save(&buffer, "PNG"));
  return bytes;
}

}  // namespace

TEST(IsAcceptableImageBody, RejectsXmlBodyWithTextXmlType)
{
  EXPECT_FALSE(CachedFileLoader::isAcceptableImageBody(serviceExceptionXml(), "text/xml"));
}

TEST(IsAcceptableImageBody, RejectsXmlBodyWithApplicationXmlType)
{
  EXPECT_FALSE(CachedFileLoader::isAcceptableImageBody(serviceExceptionXml(), "application/xml"));
  EXPECT_FALSE(
    CachedFileLoader::isAcceptableImageBody(serviceExceptionXml(), "application/vnd.ogc.se_xml"));
}

TEST(IsAcceptableImageBody, AcceptsPngWithImageType)
{
  EXPECT_TRUE(CachedFileLoader::isAcceptableImageBody(validPngBytes(), "image/png"));
}

TEST(IsAcceptableImageBody, AcceptsPngWithOddTypeDecodeDecides)
{
  // Operator decision: the header check is an early-out for declared text/XML
  // only — odd-but-valid types fall through to the decode authority.
  EXPECT_TRUE(
    CachedFileLoader::isAcceptableImageBody(validPngBytes(), "application/octet-stream"));
  EXPECT_TRUE(CachedFileLoader::isAcceptableImageBody(validPngBytes(), QString()));
}

TEST(IsAcceptableImageBody, RejectsXmlBytesMislabeledAsImage)
{
  // Header passes (image/png) but the bytes are XML — the decode gate is the
  // authority, exactly the mislabeled-server case the header check can't catch.
  EXPECT_FALSE(CachedFileLoader::isAcceptableImageBody(serviceExceptionXml(), "image/png"));
}

TEST(IsAcceptableImageBody, RejectsEmptyBody)
{
  EXPECT_FALSE(CachedFileLoader::isAcceptableImageBody(QByteArray(), "image/png"));
}

TEST(CachedFileClient, DefaultDoesNotExpectImage)
{
  // WMTS capabilities XML loads through the same shared loader — the default
  // must stay ungated or WMTS layer discovery breaks.
  CachedFileClient client;
  EXPECT_FALSE(client.expectsImage());
}

TEST(CachedFileClient, OptInExpectsImage)
{
  // What CachedTileLoader::load() constructs for tile fetches.
  CachedFileClient client(nullptr, true);
  EXPECT_TRUE(client.expectsImage());
}
