
    m_coverage_subscriber = nh.subscribe("coverage", 10, &ROSLink::coverageCallback, this);
    m_ping_subscriber = nh.subscribe("mbes_ping", 10, &ROSLink::pingCallback, this);


void ROSDetails::on_stopPingingPushButton_clicked(bool checked)
{
    qDebug() << "ROSDetails stop pinging";
    m_rosLink->sendCommand("sonar_control 0 -1");
}

void ROSDetails::on_startPingingPushButton_clicked(bool checked)
{
    qDebug() << "ROSDetails start pinging";
    m_rosLink->sendCommand("sonar_control 1 -1");
}

void ROSDetails::on_pingAndLogPushButton_clicked(bool checked)
{
    qDebug() << "ROSDetails ping and log";
    m_rosLink->sendCommand("sonar_control 2 -1");
}

void ROSDetails::on_incrementLinePushButton_clicked(bool checked)
{
    qDebug() << "ROSDetails increment line";
    m_rosLink->sendCommand("sonar_control 3 -1");
}

void ROSLink::pingCallback(const sensor_msgs::PointCloud::ConstPtr& message)
{
    QList<QGeoCoordinate> ping;
    QList<QPointF> local_ping;
//     for(auto gp: message->poses)
//     {
//         QGeoCoordinate gc;
//         gc.setLatitude(gp.pose.position.latitude);
//         gc.setLongitude(gp.pose.position.longitude);
//         ping.append(gc);
//         local_ping.append(geoToPixel(gc,autonomousVehicleProject())-m_local_reference_position);
//     }
//     QMetaObject::invokeMethod(this,"addPing", Qt::QueuedConnection, Q_ARG(QList<QGeoCoordinate>, ping), Q_ARG(QList<QPointF>, local_ping));
}


void ROSLink::updateCoverage(QList<QList<QGeoCoordinate> > coverage, QList<QPolygonF> local_coverage)
{
    prepareGeometryChange();
    m_coverage = coverage;
    m_local_coverage = local_coverage;
    update();
}

void ROSLink::addPing(QList<QGeoCoordinate> ping, QList<QPointF> local_ping)
{
    prepareGeometryChange();
    m_pings.append(ping);
    m_local_pings.append(local_ping);
    update();
}

QPainterPath ROSLink::coverageShape() const
{
    QPainterPath ret;
    for(auto p: m_local_coverage)
        ret.addPolygon(p);
//     QPolygonF poly;
//     if(!m_local_coverage.empty())
//     {
//         auto p = m_local_coverage.begin();
//         poly << *p;
//         //ret.moveTo(*p);
//         p++;
//         while(p != m_local_coverage.end())
//         {
//             poly << *p;
//             //ret.lineTo(*p);
//             p++;
//         }
//         ret.addPolygon(poly);
//         //ret.lineTo(m_local_coverage.front());
//     }
    
    return ret;
}

QPainterPath ROSLink::pingsShape() const
{
    QPainterPath ret;
    QPolygonF poly;
    for(auto p: m_local_pings)
        if(!p.empty())
        {
            auto pp = p.begin();
            poly << *pp;
            //ret.moveTo(*p);
            pp++;
            while(pp != p.end())
            {
                poly << *pp;
                //ret.lineTo(*p);
                pp++;
            }
            ret.addPolygon(poly);
            //ret.lineTo(m_local_coverage.front());
        }
    
    return ret;
}


void ROSLink::coverageCallback(const geographic_msgs::GeoPath::ConstPtr& message)
{
    QList<QList<QGeoCoordinate> > coverage;
    QList<QPolygonF> local_coverage;
    coverage.push_back(QList<QGeoCoordinate>());
    local_coverage.push_back(QPolygonF());
    for(auto gp: message->poses)
    {
        QGeoCoordinate gc;
        gc.setLatitude(gp.pose.position.latitude);
        gc.setLongitude(gp.pose.position.longitude);
        if(gc.isValid())
        {
            coverage.back().append(gc);
            local_coverage.back().append(geoToPixel(gc,autonomousVehicleProject()));
        }
        else
        {
            coverage.push_back(QList<QGeoCoordinate>());
            local_coverage.push_back(QPolygonF());
        }
    }
    QMetaObject::invokeMethod(this,"updateCoverage", Qt::QueuedConnection, Q_ARG(QList<QList<QGeoCoordinate> >, coverage), Q_ARG(QList<QPolygonF>, local_coverage));
}
