const submitButton = $("#createDelivery-submit");
/**
 * The JavaScript AJAX call for when a new delivery is created (i.e. the submit button is clicked).
 */

submitButton.click((e) => {
    e.preventDefault();
    const dataDictionary = {};
    dataDictionary["source"] = $("#source").val();
    dataDictionary["destination"] = $("#destination").val();
    const formData = JSON.stringify(dataDictionary);

    console.log(formData);
    console.log("Got here");

    $.ajax({
        type: "POST",
        url: "/api/v1/createDelivery",
        data: formData,
        dataType: "json",
        contentType: "application/json",
        success: function (res) {
            console.log('Delivery created successfully');
            if (res) {
                setToast("success", "", "Successfully created delivery", true);
                window.location.href = `/status/${res["deliveryId"]}`;
            }
        },
        error: function (xhr, status, error) {
            // error handling
            console.error('Error creating delivery:', error);
            setToast("error", "Something went wrong", "Could not create delivery");
        }
    })

});