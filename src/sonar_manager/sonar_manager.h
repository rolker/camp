
public:
    QPainterPath coverageShape() const;
    QPainterPath pingsShape() const;


public slots:
    void updateCoverage(QList<QList<QGeoCoordinate> > coverage, QList<QPolygonF> local_coverage);
    void addPing(QList<QGeoCoordinate> ping, QList<QPointF> local_ping);


private slots:
    void on_stopPingingPushButton_clicked(bool checked);
    void on_startPingingPushButton_clicked(bool checked);
    void on_pingAndLogPushButton_clicked(bool checked);
    void on_incrementLinePushButton_clicked(bool checked);

private:
    void coverageCallback(const geographic_msgs::GeoPath::ConstPtr& message);
    void pingCallback(const sensor_msgs::PointCloud::ConstPtr& message);

    ros::Subscriber m_coverage_subscriber;
    ros::Subscriber m_ping_subscriber;

    QList<QList<QGeoCoordinate> > m_coverage;
    QList<QPolygonF> m_local_coverage;

    QList<QList<QGeoCoordinate> > m_pings;
    QList<QList<QPointF> > m_local_pings;
