#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 12:11:40 2020

@author: pi
"""
from climacell_api.client import ClimacellApiClient

key = 'NpJwt2P6KpNr35DPUobnGQcXRZKZdLpo'
client = ClimacellApiClient(key)

r= client.realtime(lat=40, lon=50, fields=['temp', 'wind_gust'])

r.json()
data = r.data()

obs = data.observation_time
m = data.measurements

def realtime(self, lat, lon, fields, units='si'):
    """
    The realtime data returns up to the minute observational data for
    specificed location.

    :param float lat: Latitude of location
    :param float lon: Longitude of location
    :param list fields: List of data fields to pull
    :param string units: Either scientific ('si') or US ('us')

    :returns: Request response object with data() as ObservationData
    :rtype: ClimacellResponse
    """

    params = {
        "lat": lat,
        "lon": lon,
        "unit_system": units,
        "fields": ",".join(fields),
        "apikey": self.key
    }

    response = self._make_request(
            url_suffix="/weather/realtime", params=params)
    return ClimacellResponse(request_response=response, fields=fields,
                             response_type='realtime')

